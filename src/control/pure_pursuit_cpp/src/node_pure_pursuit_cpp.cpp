#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
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

    Point() : x(0), y(0) {}

    Point(double x, double y) : x(x), y(y) {}
};

class PurePursuit : public rclcpp::Node {
   private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr driving_command_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr rvwp_pub;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<Point> path{};

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener;

    double cog2axle{0.5};

    double Kp_ang;
    double lookahead;
    double squared_lookahead;  // allows us to skip performing sqrts in get_rvwp()
    double vel_max;
    double vel_min;
    double discovery_lookahead;
    double squared_discovery_lookahead;  // allows us to skip performing sqrts in get_rvwp()
    double discovery_vel_max;
    double discovery_vel_min;

    int fallback_path_points_offset;  // The number of path points to be skipped for an approximate rvwp

    // Internal states
    bool driving{false};
    bool following{false};

    double calc_steering(const Pose& pose, const Point& rvwp) const {
        // Calculates the steering angle based on the pose of the car and the RVWP.
        // Gets the angle between the car and the RVWP and calculates the error between the desired heading and the
        // current heading. The steering angle is then calculated using the error and the proportional gain.

        double des_heading_ang = atan2(rvwp.y - pose.y, rvwp.x - pose.x);
        double error = wrap_to_pi(pose.yaw - des_heading_ang);
        double deg_error = error * 180.0 / M_PI;
        return deg_error * Kp_ang;
    }

    double calc_velocity(double desired_steering) const {
        // Calculates the velocity based on the steering angle.
        // Reduces the velocity as the steering angle increases

        double vel =
            vel_min + std::max((vel_max - vel_min) * (1 - std::pow(std::abs(desired_steering) / 90.0, 2.0)), 0.0);
        return vel;
    }

    Pose get_wheel_position(double x, double y, double th) const {
        // Gets the position of the steering axle from the car's
        // center of gravity and heading

        double x_axle = x + cos(th) * this->cog2axle;
        double y_axle = y + sin(th) * this->cog2axle;

        return {x_axle, y_axle, th};
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
        Point closest_path_point = closest_index > -1 ? path.at(closest_index) : Point(car_axle_pos.x, car_axle_pos.y);
        if (closest_index == -1) RCLCPP_WARN(get_logger(), "Could not find closest point, have used car's axle pos");

        auto last_dist = DBL_MAX;
        int rvwp_index = -1;
        for (size_t i = 0; i < path.size(); i++) {
            auto const& point = path.at(i);
            double diff_x = point.x - closest_path_point.x;
            double diff_y = point.y - closest_path_point.y;
            double dist = diff_x * diff_x + diff_y * diff_y;
            if (dist <= this->squared_lookahead || dist >= last_dist) continue;

            double a = angle(closest_path_point.x, closest_path_point.y, point.x, point.y);
            double error = wrap_to_pi(car_axle_pos.yaw - a);
            if (error > -M_PI_2 && error < M_PI_2) {
                last_dist = dist;
                rvwp_index = i;
            }
        }

        if (rvwp_index == -1) {
            // Handle path points size overflow.
            int path_points_count = path.size() - 1;
            if (closest_index + fallback_path_points_offset > path_points_count) {
                rvwp_index = abs(path_points_count - (closest_index + fallback_path_points_offset));
            } else {
                rvwp_index = closest_index + fallback_path_points_offset;
            }
        }
        return path.at(rvwp_index);
    }

    bool can_drive() const { return this->driving && this->path.size() > 0; }

    void timer_callback() {
        // Listens to odom->base_footprint transform and updates the state.
        // Solution is to get the delta and add it to the previous state and subtract the delta from the previous
        // map->odom transform.
        if (!this->can_drive()) {
            return;
        }
        RCLCPP_INFO_ONCE(get_logger(), "Can drive, starting pursuit");

        auto start_time = std::chrono::high_resolution_clock::now();

        geometry_msgs::msg::TransformStamped odom_to_base;
        try {
            odom_to_base = buffer.lookupTransform("track", "base_footprint", tf2::TimePointZero);
        } catch (tf2::TransformException const& e) {
            RCLCPP_WARN(get_logger(), "Transform Exception: %s", e.what());
            return;
        }

        tf2::Quaternion q(odom_to_base.transform.rotation.x, odom_to_base.transform.rotation.y,
                          odom_to_base.transform.rotation.z, odom_to_base.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        Pose position =
            get_wheel_position(odom_to_base.transform.translation.x, odom_to_base.transform.translation.y, yaw);

        double steering_angle = 0.0;
        double velocity = 0.0;

        // try {
        //     // Check if the car is behind the RVWP
        //     Point rvwp = get_rvwp(position);
        //     if (rvwp.x < position.x) {
        //         throw std::logic_error("RVWP is behind the car");
        //     }
        //     steering_angle = calc_steering(position, rvwp);
        //     velocity = calc_velocity(steering_angle);

        //     // publish rvwp
        //     geometry_msgs::msg::PointStamped rvwp_msg;
        //     rvwp_msg.header.stamp = now();
        //     rvwp_msg.header.frame_id = "track";
        //     rvwp_msg.point.x = rvwp.x;
        //     rvwp_msg.point.y = rvwp.y;
        //     this->rvwp_pub->publish(rvwp_msg);

        // } catch (std::logic_error const& e) {
        //     RCLCPP_ERROR(get_logger(), "Logic Error: %s", e.what());
        // }
        // Check if the car is behind the RVWP
        Point rvwp = get_rvwp(position);
        steering_angle = calc_steering(position, rvwp);
        velocity = calc_velocity(steering_angle);

        // publish rvwp
        geometry_msgs::msg::PointStamped rvwp_msg;
        rvwp_msg.header.stamp = now();
        rvwp_msg.header.frame_id = "track";
        rvwp_msg.point.x = rvwp.x;
        rvwp_msg.point.y = rvwp.y;
        this->rvwp_pub->publish(rvwp_msg);

        // publish drive message
        ackermann_msgs::msg::AckermannDriveStamped driving_command;
        driving_command.header.stamp = now();
        driving_command.drive.speed = velocity;
        driving_command.drive.steering_angle = steering_angle;
        this->driving_command_pub->publish(driving_command);

        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = now - start_time;
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Elapsed time: %f", duration.count());
    }

    void path_callback(nav_msgs::msg::Path const& spline_path_msg) {
        double distance = 0.0;
        path.clear();
        for (const auto& pose : spline_path_msg.poses) {
            if (path.size() >= 2) {
                auto const& previous_point = path.at(path.size() - 1);
                auto diff_x = previous_point.x - pose.pose.position.x;
                auto diff_y = previous_point.y - pose.pose.position.y;
                distance += sqrt(diff_x * diff_x + diff_y * diff_y);
            }
            path.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }
        // Calculate the number of path points to skip when finding a fallback RVWP.
        // Formula is the number of path points divided by the calculated distance in metres, giving the number of
        // points per metre, which is then multiplied by the lookahead value (also in metres) giving the number of
        // path points that shuld be skipped.
        fallback_path_points_offset = round(path.size() / distance * lookahead);
    }

    void state_callback(driverless_msgs::msg::State const& msg) {
        if (msg.state == driverless_msgs::msg::State::DRIVING) {
            this->driving = true;
            RCLCPP_INFO_ONCE(get_logger(), "Ready to drive");
        }
        if (msg.lap_count == 0) {
            this->following = false;
            this->vel_max = this->get_parameter("discovery_vel_max").as_double();
            this->vel_min = this->get_parameter("discovery_vel_min").as_double();
            this->lookahead = this->get_parameter("discovery_lookahead").as_double();
        }
        if (msg.lap_count > 0 && !this->following) {
            this->following = true;
            this->vel_max = this->get_parameter("vel_max").as_double();
            this->vel_min = this->get_parameter("vel_min").as_double();
            this->lookahead = this->get_parameter("lookahead").as_double();
            RCLCPP_INFO(get_logger(), "Discovery lap completed, commencing following");
        }
    }

    void initialise_params() {
        this->declare_parameter<double>("Kp_ang", -3.0);
        this->declare_parameter<double>("vel_max", 5.0);
        this->declare_parameter<double>("vel_min", 2.0);
        this->declare_parameter<double>("lookahead", 3.0);
        this->declare_parameter<double>("discovery_lookahead", 3.5);
        this->declare_parameter<double>("discovery_vel_max", 3.0);
        this->declare_parameter<double>("discovery_vel_min", 2.0);

        this->Kp_ang = this->get_parameter("Kp_ang").as_double();
        this->lookahead = this->get_parameter("discovery_lookahead").as_double();
        this->vel_max = this->get_parameter("discovery_vel_max").as_double();
        this->vel_min = this->get_parameter("discovery_vel_min").as_double();

        this->squared_lookahead = lookahead * lookahead;
    }

   public:
    PurePursuit() : Node("pure_pursuit_cpp_node"), buffer(this->get_clock()), listener(buffer) {
        this->initialise_params();

        this->path_sub = this->create_subscription<nav_msgs::msg::Path>(
            "/planning/midline_path", QOS_LATEST, std::bind(&PurePursuit::path_callback, this, _1));
        this->state_sub = this->create_subscription<driverless_msgs::msg::State>(
            "/system/as_status", QOS_LATEST, std::bind(&PurePursuit::state_callback, this, _1));
        timer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&PurePursuit::timer_callback, this));

        this->driving_command_pub =
            this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/control/driving_command", 10);
        this->rvwp_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/control/rvwp", 10);

        RCLCPP_INFO(get_logger(), "---Pure Pursuit (C++) Node Initialised---");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
