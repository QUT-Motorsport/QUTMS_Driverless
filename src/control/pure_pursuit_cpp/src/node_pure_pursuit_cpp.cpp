#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/path_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

struct Pose {
    double const x;
    double const y;
    double const yaw;

    Pose(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
};

struct Point {
    double const x;
    double const y;
    double const turn_intensity;

    Point() : x(0), y(0), turn_intensity(0) {}

    Point(double x, double y, double turn_intensity) : x(x), y(y), turn_intensity(turn_intensity) {}
};

class Fast : public rclcpp::Node {
   private:
    rclcpp::Subscription<driverless_msgs::msg::PathStamped>::SharedPtr path_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr slam_pose_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr driving_command_pub;
    std::vector<Point> path{};
    double lookahead;
    double squared_lookahead;  // allows us to skip performing sqrts in get_rvwp()
    double kp_ang;
    double target_velocity;
    int fallback_path_points_offset;  // The number of path points to be skipped for an approximate rvwp
    size_t count{0};

    static Pose calc_car_axle_pos(double car_pos_x, double car_pos_y, double heading) {
        constexpr double cog2axle = 0.5;
        double x = car_pos_x + cos(heading) * cog2axle;
        double y = car_pos_y + sin(heading) * cog2axle;
        return {x, y, heading};
    }

    Point get_rvwp(Pose const& car_axle_pos) const {
        // Find the path point closest the to position of the car's axle.
        auto closest_dist = DBL_MAX;
        int closest_index = -1;
        for (size_t i = 0; i < path.size(); i++) {
            auto const& point = path.at(i);
            double diff_x = point.x - car_axle_pos.x;
            double diff_y = point.y - car_axle_pos.y;
            double dist = diff_x * diff_x + diff_y * diff_y;
            if (dist < closest_dist) {
                closest_dist = dist;
                closest_index = i;
            }
        }

        // Note that it is logically impossible for closest_index to be -1 (no closest point found) as this method
        // is called after a path.size() > 0 check.
        Point closest_path_point =
            closest_index > -1 ? path.at(closest_index) : Point(car_axle_pos.x, car_axle_pos.y, 0);
        if (closest_index == -1) RCLCPP_WARN(get_logger(), "Could not find closest point, have used car's axle pos");

        auto rvwp_dist = DBL_MAX;
        int rvwp_index = -1;
        for (size_t i = 0; i < path.size(); i++) {
            auto const& point = path.at(i);
            double diff_x = point.x - closest_path_point.x;
            double diff_y = point.y - closest_path_point.y;
            double dist = diff_x * diff_x + diff_y * diff_y;
            if (dist <= squared_lookahead || dist >= rvwp_dist) continue;

            double a = angle(closest_path_point.x, closest_path_point.y, point.x, point.y);
            double error = wrap_to_pi(car_axle_pos.yaw - a);
            if (error > -M_PI_2 && error < M_PI_2) {
                rvwp_dist = dist;
                rvwp_index = i;
            }
        }

        if (rvwp_index == -1 || rvwp_index == closest_index) {
            RCLCPP_WARN(get_logger(), "No valid RVWP found, using fallback point");
            // Handle path points size overflow.
            int path_points_count = path.size() - 1;
            if (closest_index + fallback_path_points_offset > path_points_count)
                rvwp_index = abs(path_points_count - (closest_index + fallback_path_points_offset));
            else
                rvwp_index = closest_index + fallback_path_points_offset;
        }

        return path.at(rvwp_index);
    }

    void path_callback(driverless_msgs::msg::PathStamped const& spline_path_msg) {
        double distance = 0.0;
        path.clear();
        for (const auto& point : spline_path_msg.path) {
            if (path.size() >= 2) {
                auto const& previous_point = path.at(path.size() - 1);
                auto diff_x = previous_point.x - point.location.x;
                auto diff_y = previous_point.y - point.location.y;
                distance += sqrt(diff_x * diff_x + diff_y * diff_y);
            }
            path.emplace_back(point.location.x, point.location.y, point.turn_intensity);
        }
        // Calculate the number of path points to skip when finding a fallback RVWP.
        // Formula is the number of path points divided by the calculated distance in metres, giving the number of
        // points per metre, which is then multiplied by the lookahead value (also in metres) giving the number of
        // path points that shuld be skipped.
        fallback_path_points_offset = round(path.size() / distance * lookahead);
    }

    void pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped const& pose_msg) {
        if (path.empty()) return;
        auto start_time = std::chrono::high_resolution_clock::now();

        tf2::Quaternion q(pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y,
                          pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        double position_cog_x = pose_msg.pose.pose.position.x;
        double position_cog_y = pose_msg.pose.pose.position.y;
        Pose car_axle_position = calc_car_axle_pos(position_cog_x, position_cog_y, yaw);
        Point rvwp = get_rvwp(car_axle_position);

        double desired_heading_angle = angle(car_axle_position.x, car_axle_position.y, rvwp.x, rvwp.y);
        double error = wrap_to_pi(yaw - desired_heading_angle);
        double steering_angle = (error * (180 / M_PI)) * kp_ang;

        ackermann_msgs::msg::AckermannDriveStamped driving_command;
        driving_command.header.stamp = now();
        driving_command.drive.speed = target_velocity;
        driving_command.drive.steering_angle = steering_angle;
        this->driving_command_pub->publish(driving_command);

        count++;
        if (count == 50) {
            count = 0;
            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = now - start_time;
            RCLCPP_INFO(get_logger(), "Elapsed time: %f", duration.count());
        }
    }

   public:
    Fast() : Node("pure_pursuit_cpp_node") {
        this->path_sub = this->create_subscription<driverless_msgs::msg::PathStamped>(
            "/planner/path", QOS_LATEST, std::bind(&Fast::path_callback, this, _1));
        this->slam_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/slam/car_pose", QOS_LATEST, std::bind(&Fast::pose_callback, this, _1));

        this->driving_command_pub =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control/driving_command", 10);

        this->target_velocity = this->declare_parameter<double>("target_velocity", 10.0);
        this->kp_ang = this->declare_parameter<double>("kp_ang", -3.0);
        this->lookahead = this->declare_parameter<double>("lookahead", 3.0);
        squared_lookahead = lookahead * lookahead;
        RCLCPP_INFO(get_logger(), "---Fast Node Initialised---");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Fast>());
    rclcpp::shutdown();
    return 0;
}
