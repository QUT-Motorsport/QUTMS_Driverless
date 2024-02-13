#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <unsupported/Eigen/Splines>
#include <vector>

#include "driverless_common/common.hpp"
#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "driverless_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"

using namespace std;
using namespace std::chrono_literals;

class OrderedMapSpline : public rclcpp::Node {
   public:
    OrderedMapSpline();

   private:
    // declare pubs and subs
    rclcpp::Subscription<driverless_msgs::msg::ConeDetectionStamped>::SharedPtr mapSubscription;
    rclcpp::Subscription<driverless_msgs::msg::State>::SharedPtr stateSubscription;
    rclcpp::TimerBase::SharedPtr planningTimer;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blueBoundPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr yellowBoundPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plannedPathPublisher;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapPublisher;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr mapMetaPublisher;

    // declare variables
    int splineConst{10};
    int segment{1};
    bool following{false};
    driverless_msgs::msg::ConeDetectionStamped::SharedPtr currentTrack;
    int interpConeNum{3};
    bool finalPathPublished{false};
    double startDist{1.0};
    double searchRange{5.0};
    double searchAngle{M_PI / 4};

    void OrderedMapSpline::stateCallback(const driverless_msgs::msg::State::SharedPtr msg) {
        if (msg->state == driverless_msgs::msg::State::DRIVING && msg->lap_count > 0 && !following) {
            following = true;
            RCLCPP_INFO(get_logger(), "Lap completed, planning commencing");
        }
    }

    void OrderedMapSpline::mapCallback(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr trackMsg);
};
