#include <math.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>
#include <rclcpp/qos.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "driverless_msgs/msg/mpc_centerline.hpp"
#include "driverless_msgs/msg/mpc_path_point.hpp"
#include "driverless_msgs/msg/mpc_state.hpp"
#include "driverless_msgs/msg/path_stamped.hpp"
#include "fs_msgs/msg/wheel_states.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

double wrap_angle(double angle) {
    return (angle - 2 * M_PI * floor(angle / (2 * M_PI)) - M_PI) *
           -1;  // wrap angle - don't ask how this equation works i'm only using it because fmod() in c sucks
}

float wrap_angle(float angle) { return (float)wrap_angle((double)angle); }

class MPCTransforms : public rclcpp::Node {
    rclcpp::Subscription<driverless_msgs::msg::PathStamped>::SharedPtr track_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<fs_msgs::msg::WheelStates>::SharedPtr steering_sub;
    rclcpp::Publisher<driverless_msgs::msg::MPCCenterline>::SharedPtr track_pub;
    rclcpp::Publisher<driverless_msgs::msg::MPCState>::SharedPtr state_pub;
    driverless_msgs::msg::MPCCenterline track;
    driverless_msgs::msg::MPCState state;

    nav_msgs::msg::Odometry state_inertial;

    float last_loc_duration = 0;

   public:
    MPCTransforms() : Node("mpc_transforms") {
        rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        track_sub = this->create_subscription<driverless_msgs::msg::PathStamped>(
            "/path_planner/path", qos_, std::bind(&MPCTransforms::track_callback, this, _1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/testing_only/odom", qos_, std::bind(&MPCTransforms::odom_callback, this, _1));

        steering_sub = this->create_subscription<fs_msgs::msg::WheelStates>(
            "/fsds/wheel_states", qos_, std::bind(&MPCTransforms::steering_callback, this, _1));

        track_pub = this->create_publisher<driverless_msgs::msg::MPCCenterline>("mpc/inputs/track", qos_);

        state_pub = this->create_publisher<driverless_msgs::msg::MPCState>("mpc/inputs/state", qos_);

        RCLCPP_INFO(this->get_logger(), "mpc transformer start");
    }

   private:
    void track_callback(driverless_msgs::msg::PathStamped track_inertial) {
        int num_points = track_inertial.path.size();
        RCLCPP_INFO(this->get_logger(), "track received");
        RCLCPP_INFO(this->get_logger(), "points: %d", num_points);
        // RCLCPP_INFO(this->get_logger(), "first (%f, %f)", track_inertial.path[0].location.x,
        // track_inertial.path[0].location.y); RCLCPP_INFO(this->get_logger(), "last (%f, %f)\n",
        // track_inertial.path[num_points-1].location.x, track_inertial.path[num_points-1].location.y);

        auto start = std::chrono::high_resolution_clock::now();

        track = driverless_msgs::msg::MPCCenterline();
        driverless_msgs::msg::MPCPathPoint mpp;
        mpp.location = track_inertial.path[0].location;
        track.path.push_back(mpp);

        float s = 0;
        for (int i = 1; i < num_points - 1; i++) {
            geometry_msgs::msg::Point p1 = track_inertial.path[i - 1].location;
            geometry_msgs::msg::Point p2 = track_inertial.path[i].location;
            geometry_msgs::msg::Point p3 = track_inertial.path[i + 1].location;
            s += sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

            mpp = driverless_msgs::msg::MPCPathPoint();
            mpp.location = p2;
            mpp.curvature = curvature(p1, p2, p3);
            mpp.s = s;
            mpp.track_angle = atan2(p3.y - p1.y, p3.x - p1.x);

            track.path.push_back(mpp);
        }

        track.path[0].curvature = track.path[1].curvature;
        track.path[0].track_angle = track.path[1].track_angle;

        mpp = driverless_msgs::msg::MPCPathPoint();
        mpp.location = track_inertial.path[num_points - 1].location;
        mpp.curvature = track.path[num_points - 2].curvature;
        mpp.s = track.path[num_points - 2].s;
        ;
        mpp.track_angle = track.path[num_points - 2].track_angle;
        ;
        track.path.push_back(mpp);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        RCLCPP_INFO(this->get_logger(), "%f seconds total for path serial",
                    std::chrono::duration<double>(duration).count());
        RCLCPP_INFO(this->get_logger(), "%f seconds total for localise serial", last_loc_duration);

        // RCLCPP_INFO(this->get_logger(), "printing state vec\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f",
        //                                 state.s,
        //                                 state.ey,
        //                                 state.etheta,
        //                                 state.otheta_dot,
        //                                 state.vx,
        //                                 state.vy,
        //                                 state.d,
        //                                 state.t.sec + state.t.nanosec * 10e-9);

        // for(int i = 0; i < num_points; i++)
        // {
        //     RCLCPP_INFO(this->get_logger(), "%d - %f\t- (%f, %f, %f)",
        //                 i,
        //                 track.path[i].curvature,
        //                 track.path[i].location.x,
        //                 track.path[i].location.y,
        //                 track.path[i].s);
        // }

        track_pub->publish(track);
        // trigger track_localise() to place car on track
        this->track_localise();
    }

   public:
    static float curvature(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2, geometry_msgs::msg::Point p3) {
        float x1 = p1.x;
        float y1 = p1.y;
        float x2 = p2.x;
        float y2 = p2.y;
        float x3 = p3.x;
        float y3 = p3.y;

        float dx1 = x2 - x1;
        float dy1 = y2 - y1;
        float dx2 = x3 - x1;
        float dy2 = y3 - y1;
        float area = 0.5 * (dx1 * dy2 - dy1 * dx2);

        float len0 = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
        float len1 = sqrt(pow(x2 - x3, 2) + pow(y2 - y3, 2));
        float len2 = sqrt(pow(x1 - x3, 2) + pow(y1 - y3, 2));

        return 4 * area / (len0 * len1 * len2);
    }

   public:
    void odom_callback(nav_msgs::msg::Odometry odom) {
        // RCLCPP_INFO(this->get_logger(), "odom received");
        // update some states
        state.vx = state_inertial.twist.twist.linear.x;
        state.vy = state_inertial.twist.twist.linear.y;
        state.otheta_dot = state_inertial.twist.twist.angular.z;
        state.otheta_dot = state_inertial.twist.twist.angular.z;
        state.t = odom.header.stamp;
        // get car pos & vel in world space
        state_inertial = odom;
        // trigger track_localise() to place car on track
        this->track_localise();
    }

   public:
    void steering_callback(fs_msgs::msg::WheelStates wheel_msg) {
        // this does not correctly calculate the correct steering angle but it is close enough for now
        // see - https://www.xarg.org/book/kinematics/ackerman-steering/
        state.d = (wheel_msg.fl_steering_angle + wheel_msg.fr_steering_angle) / 2;
    }

   private:
    void track_localise() {
        if (track.path.size() <= 0) {
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "localising car");
        // use car world pos to find s
        geometry_msgs::msg::Point car_loc = state_inertial.pose.pose.position;
        // conversion from quaternion (state_inertial.pose.pose.orientation) to theta (rotation about z axis) assumes
        // that all rotation is about the z axis this is mostly true
        float car_dir = -asin(state_inertial.pose.pose.orientation.w) * 2 * state_inertial.pose.pose.orientation.z /
                        abs(state_inertial.pose.pose.orientation.z);

        auto start = std::chrono::high_resolution_clock::now();
        // use s to convert world pos & vel to track pos & vel
        int num_points = track.path.size();
        float best_dist = INFINITY;
        int best_index = 0;
        for (int i = 0; i < num_points; i++) {
            float dx = car_loc.x - track.path[i].location.x;
            float dy = car_loc.y - track.path[i].location.y;
            float dist = sqrt(pow(dx, 2) + pow(dy, 2));
            // RCLCPP_INFO(this->get_logger(), "|%f i - %f j| = %f", dx, dy, dist);
            if (dist < best_dist) {
                best_dist = dist;
                best_index = i;
            }
        }

        float test_angle = atan2(car_loc.y - track.path[best_index].location.y,
                                 car_loc.x - track.path[best_index].location.x);  // angle from closest point to car
        test_angle -= track.path[best_index].track_angle;                         // rel angle from closest point to car
        test_angle = wrap_angle(test_angle);
        if (test_angle < 0) {
            best_dist *= -1;
        }

        // RCLCPP_INFO(this->get_logger(), "best %d @ %f", best_index, best_dist);

        state.s = track.path[best_index].s;
        state.ey = best_dist;  // this needs to be signed
        state.etheta = -wrap_angle(car_dir - track.path[best_index].track_angle);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        last_loc_duration = std::chrono::duration<double>(duration).count();
        // RCLCPP_INFO(this->get_logger(), "%f seconds total for localise serial", last_loc_duration);
        state_pub->publish(state);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCTransforms>());
    rclcpp::shutdown();
    return 0;
}
