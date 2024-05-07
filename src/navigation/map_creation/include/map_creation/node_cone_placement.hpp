#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace map_creation {

struct TrackedCone {
    float local_x;
    float local_y;
    float map_x;
    float map_y;
    std::array<std::array<double, 2>, 2> cov;
    int colour;
    std::string sensor;
    bool confirmed{false};
    int frame_count{0};
    int yellow_count{0};
    int blue_count{0};
    int orange_count{0};

    TrackedCone(const driverless_msgs::msg::Cone &cone, const std::string &frame_id, const std::vector<double> &pose) {
        // TODO: offset these with transform lookup
        if (frame_id == "velodyne") {
            local_x = cone.location.x + 1.65;
            sensor = "lidar";
        } else {
            local_x = cone.location.x - 0.2;
            sensor = "camera";
        }
        local_y = cone.location.y;
        colour = cone.color;
        frame_count = 1;

        // transform detection to map
        std::array<std::array<double, 2>, 2> rotation_mat = {
            {{cos(pose[2]), -sin(pose[2])}, {sin(pose[2]), cos(pose[2])}}};
        std::array<double, 2> map_coords = {{rotation_mat[0][0] * local_x + rotation_mat[0][1] * local_y + pose[0],
                                             rotation_mat[1][0] * local_x + rotation_mat[1][1] * local_y + pose[1]}};
        map_x = map_coords[0];
        map_y = map_coords[1];
    }

    void update(const TrackedCone &other, const std::vector<double> &pose) {
        // update position with lidar
        if (other.sensor == "lidar") {
            // update the location of the cone using the average of the previous location and the new location
            map_x = (map_x * frame_count + other.map_x) / (frame_count + 1);
            map_y = (map_y * frame_count + other.map_y) / (frame_count + 1);
            // update the count of the cone
            frame_count += 1;
        }

        // update colour with camera
        else {
            // update the colour weight of the cone
            if (other.colour == driverless_msgs::msg::Cone::YELLOW) {
                yellow_count += 1;
            } else if (other.colour == driverless_msgs::msg::Cone::BLUE) {
                blue_count += 1;
            } else if (other.colour == driverless_msgs::msg::Cone::ORANGE_BIG) {
                orange_count += 1;
            }

            if (yellow_count > blue_count && yellow_count > orange_count) {
                colour = driverless_msgs::msg::Cone::YELLOW;
            } else if (blue_count > yellow_count && blue_count > orange_count) {
                colour = driverless_msgs::msg::Cone::BLUE;
            } else if (orange_count > yellow_count && orange_count > blue_count) {
                colour = driverless_msgs::msg::Cone::ORANGE_SMALL;
            }

            // if (orange_count > 10 || orange_count > yellow_count || orange_count > blue_count) {
            //     colour = driverless_msgs::msg::Cone::ORANGE_BIG;
            // }
        }

        // transform map to car
        std::array<std::array<double, 2>, 2> rotation_mat = {
            {{cos(-pose[2]), -sin(-pose[2])}, {sin(-pose[2]), cos(-pose[2])}}};
        std::array<double, 2> local_coords = {{rotation_mat[0][0] * map_x + rotation_mat[0][1] * map_y + pose[0],
                                               rotation_mat[1][0] * map_x + rotation_mat[1][1] * map_y + pose[1]}};
        local_x = local_coords[0];
        local_y = local_coords[1];

        // update covariance based on the number of detections
        cov = {{{5.0 / frame_count, 0.0}, {0.0, 5.0 / frame_count}}};
    }

    std::array<double, 4UL> cov_as_msg() const {
        std::array<double, 4UL> cov_flat = {cov[0][0], cov[0][1], cov[1][0], cov[1][1]};
        return cov_flat;
    }

    driverless_msgs::msg::Cone cone_as_msg() const {
        driverless_msgs::msg::Cone cone_msg;
        cone_msg.location.x = map_x;
        cone_msg.location.y = map_y;
        cone_msg.location.z = 0.0;
        cone_msg.color = colour;
        return cone_msg;
    }

    driverless_msgs::msg::Cone local_cone_as_msg() const {
        driverless_msgs::msg::Cone cone_msg;
        cone_msg.location.x = local_x;
        cone_msg.location.y = local_y;
        cone_msg.location.z = 0.0;
        cone_msg.color = colour;
        return cone_msg;
    }
};

class ConeAssociation : public rclcpp::Node {
   public:
    ConeAssociation();

   private:
    rclcpp::Subscription<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr lidar_subscriber;
    rclcpp::Subscription<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr vision_subscriber;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr state_subscriber;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr global_publisher;
    rclcpp::Publisher<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr local_publisher;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    std::vector<double> pose;
    std::vector<TrackedCone> track;
    std::chrono::time_point<std::chrono::system_clock> last_time{std::chrono::system_clock::now()};

    bool mapping{false};

    double view_x;
    double view_y;
    double radius;
    int min_detections;

    void state_callback(const driverless_msgs::msg::State::SharedPtr msg);
    void callback(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr msg);
    void findClosestCone(const driverless_msgs::msg::Cone &cone, std::string frame_id);
    void createConeDetections(const driverless_msgs::msg::ConeDetectionStamped &msg);
};
}  // namespace map_creation
