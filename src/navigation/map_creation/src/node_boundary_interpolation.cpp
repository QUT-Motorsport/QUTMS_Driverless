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

std::vector<std::vector<double>> approximateBSplinePath(const std::vector<double> &x, const std::vector<double> &y,
                                                        int nPathPoints, int degree, double s) {
    // Convert input vectors to Eigen::VectorXd
    Eigen::VectorXd eigenX = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    Eigen::VectorXd eigenY = Eigen::Map<const Eigen::VectorXd>(y.data(), y.size());

    // Normalize x to [0, 1]
    Eigen::VectorXd normalizedX = (eigenX - eigenX.minCoeff()) / (eigenX.maxCoeff() - eigenX.minCoeff());

    // Create a uniform B-spline with the specified degree
    typedef Eigen::Spline<double, 2> spline2d;
    Eigen::Spline<spline2d> spline = Eigen::SplineFitting<spline2d>::Interpolate(normalizedX, eigenY, degree, s);

    // Sample the B-spline to get the path points
    Eigen::VectorXd sampledX = Eigen::VectorXd::LinSpaced(nPathPoints, normalizedX.minCoeff(), normalizedX.maxCoeff());
    Eigen::VectorXd sampledY = spline(sampledX);

    // Denormalize x
    std::vector<double> splineX = (sampledX * (eigenX.maxCoeff() - eigenX.minCoeff())) + eigenX.minCoeff();
    std::vector<double> splineY = Eigen::Map<std::vector<double>>(sampledY.data(), sampledY.size());

    // Convert to x, y points
    std::vector<std::vector<double>> splinePoints;
    for (int i = 0; i < nPathPoints; ++i) {
        splinePoints.push_back({splineX[i], splineY[i]});
    }

    return splinePoints;
}

// Function to create Path message
nav_msgs::msg::Path makePathMsg(const std::vector<std::vector<double>> &points, int splineLen) {
    nav_msgs::msg::Path pathMsg;
    pathMsg.header.frame_id = "track";

    for (int i = 0; i < splineLen; ++i) {
        // get angle between current point and next point
        double th_change;
        if (i < splineLen - 1) {
            th_change = angle(points[i], points[i + 1]);
        } else if (i == splineLen - 1) {
            th_change = angle(points[i], points[0]);
        }
        // keep between 360
        if (th_change > M_PI) {
            th_change = th_change - 2 * M_PI;
        } else if (th_change < -M_PI) {
            th_change = th_change + 2 * M_PI;
        }

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "track";
        pose.pose.position.x = points[i][0];
        pose.pose.position.y = points[i][1];
        pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, th_change);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        pathMsg.poses.push_back(pose);
    }

    return pathMsg;
}

bool getClosestCone(const std::vector<std::vector<double>> &cones, int dir, int startDist,
                    double pos std::vector<double> &nearestCone, ) {
    // Initialize variables for tracking the nearest cone
    double lastDist = std::numeric_limits<double>::infinity();

    // Iterate through all cones and find the closest one
    for (const auto &cone : cones) {
        double currentDist = fast_dist({startDist, dir * pos}, cone);
        if (currentDist < lastDist) {
            nearestCone = cone;
            lastDist = currentDist;
        }
    }
    // have we found a cone?
    if (lastDist == std::numeric_limits<double>::infinity())
        return false;
    else
        return true;
}

// Function to get next cone
std::pair<std::vector<double>, double> getNextCone(const std::vector<std::vector<double>> &cones,
                                                   const std::vector<double> &currentCone, double lastAngle,
                                                   double searchRange, double searchAngle,
                                                   std::vector<double> &nearestCone, double &nearestAngle) {
    // Initialize variables for tracking the nearest cone and angle
    std::vector<double> nearestCone;
    double nearestAngle = 0.0;
    double lastDist = std::numeric_limits<double>::infinity();

    // Rotate search area by last angle
    for (const auto &cone : cones) {
        double dist = fastDist(currentCone, cone);
        double coneAngle = angle(currentCone, cone);
        double error = wrapToPi(lastAngle - coneAngle);

        if (searchAngle > error && error > -searchAngle && dist < searchRange * searchRange) {
            if (dist < lastDist) {
                nearestCone = cone;
                nearestAngle = coneAngle;
                lastDist = dist;
            }
        }
    }

    return std::make_pair(nearestCone, nearestAngle);
}

// Function to extract cones from message
std::vector<std::vector<double>> discoveryCones(const std::vector<driverless_msgs::msg::Cone> &cones) {
    std::vector<std::vector<double>> unsearchedCones;

    // Iterate through all cones in the message
    for (const auto &cone : cones) {
        // Extract x and y coordinates from the cone message
        double x = cone.location.x;
        double y = cone.location.y;

        // Add the cone's coordinates to the list
        unsearchedCones.push_back({x, y});
    }

    return unsearchedCones;
}

// // Function to create OccupancyGrid message
// nav_msgs::msg::OccupancyGrid getOccupancyGrid(
//     const vector<vector<double>> &bluePoints,
//     const vector<vector<double>> &yellowPoints,
//     const std_msgs::msg::Header &header)
// {
//     // Implementation of the function
//     // ...

//     return map;
// }

class OrderedMapSpline : public rclcpp::Node {
   public:
    OrderedMapSpline() : Node("ordered_map_spline_node") {
        // Create subscriptions, timers, and publishers
        mapSubscription = this->create_subscription<driverless_msgs::msg::ConeDetectionStamped>(
            "/slam/global_map", QOS_LATEST, std::bind(&OrderedMapSpline::mapCallback, this, std::placeholders::_1));

        stateSubscription = this->create_subscription<driverless_msgs::msg::State>(
            "/system/as_status", QOS_LATEST, std::bind(&OrderedMapSpline::stateCallback, this, std::placeholders::_1));

        blueBoundPublisher = this->create_publisher<nav_msgs::msg::Path>("/planning/blue_bounds", QOS_LATEST);
        yellowBoundPublisher = this->create_publisher<nav_msgs::msg::Path>("/planning/yellow_bounds", QOS_LATEST);
        plannedPathPublisher = this->create_publisher<nav_msgs::msg::Path>("/planning/midline_path", QOS_LATEST);

        mapPublisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        mapMetaPublisher = this->create_publisher<nav_msgs::msg::MapMetaData>(
            "/map_metadata", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    }

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

    void OrderedMapSpline::mapCallback(const driverless_msgs::msg::ConeDetectionStamped::SharedPtr trackMsg) {
        RCLCPP_DEBUG(get_logger(), "Received map");

        // if (!following) {
        //     RCLCPP_INFO_ONCE(get_logger(), "Discovering");
        //     auto unsearched_cones = discoveryCones(current_track->cones);
        // } else {
        //     RCLCPP_INFO_ONCE(get_logger(), "Following");
        //     auto mapped_cones = mappedCones(current_track->cones);
        // }
        auto unsearched_cones = discoveryCones(trackMsg->cones);

        std::vector<double> closest_blue;
        std::vector<double> closest_yellow;
        if (!getClosestCone(unsearched_cones, 1, startDist, 0.0, closest_blue)) return;
        if (!getClosestCone(unsearched_cones, -1, startDist, 0.0, closest_yellow)) return;

        // Example: Sort cones into order by finding the next cone in the direction of travel
        auto ordered_blues = std::vector<std::vector<double>>{closest_blue};
        auto ordered_yellows = std::vector<std::vector<double>>{closest_yellow};
        auto last_blue = closest_blue;
        auto last_yellow = closest_yellow;
        auto last_blue_angle = 0.0;
        auto last_yellow_angle = 0.0;

        // search cones
        while (unsearched_cones.size() > 0) {
            std::vector<double> next_blue;
            double next_blue_angle;
            if (!getNextCone(unsearched_cones, last_blue, last_blue_angle, searchRange, searchAngle, &next_blue,
                             &next_blue_angle)) {
                break;
            }

            ordered_blues.push_back(next_blue.first);
            unsearched_cones.erase(std::remove(unsearched_cones.begin(), unsearched_cones.end(), next_blue.first),
                                   unsearched_cones.end());
            last_blue = next_blue.first;
            last_blue_angle = next_blue.second;
        }

        while (unsearched_cones.size() > 0) {
            std::vector<double> next_yellow;
            double next_yellow_angle;
            if (!getNextCone(unsearched_cones, last_yellow, last_yellow_angle, searchRange, searchAngle, &next_yellow,
                             &next_yellow_angle)) {
                break;
            }

            ordered_yellows.push_back(next_yellow.first);
            unsearched_cones.erase(std::remove(unsearched_cones.begin(), unsearched_cones.end(), next_yellow.first),
                                   unsearched_cones.end());
            last_yellow = next_yellow.first;
            last_yellow_angle = next_yellow.second;
        }

        // Spline smoothing
        // make number of pts based on length of path
        int spline_len = splineConst * ordered_blues.size();

        // specify degree of spline if less than 3 cones
        int blue_degree = ordered_blues.size() - 1;
        if (ordered_blues.size() <= 3) blue_degree = 3;
        int yellow_degree = ordered_yellows.size() - 1;
        if (ordered_yellows.size() <= 3) yellow_degree = 3;

        if (blue_degree <= 1 || yellow_degree <= 1) {
            RCLCPP_WARN(get_logger(), "Not enough cones to spline");
            return;
        }

        auto yellow_pts =
            approximateBSplinePath(ordered_yellows[0], ordered_yellows[1], spline_len, yellow_degree, 0.01);
        auto blue_pts = approximateBSplinePath(ordered_blues[0], ordered_blues[1], spline_len, blue_degree, 0.01);

        std::vector<std::vector<double>> mid_pts;
        for (int i = 0; i < spline_len; ++i) {
            mid_pts.push_back(midpoint(yellow_pts[i], blue_pts[i]));
        }

        auto blue_bound_msg = makePathMsg(ordered_blues, spline_len);
        blue_bound_pub->publish(blue_bound_msg);

        auto yellow_bound_msg = makePathMsg(ordered_yellows, spline_len);
        yellow_bound_pub->publish(yellow_bound_msg);

        auto midline_msg = makePathMsg(mid_pts, spline_len);
        midline_pub->publish(midline_msg);

        // Example: Create and publish the occupancy grid
        // auto map = getOccupancyGrid(blue_points, yellow_points, current_track->header);
        // map_pub->publish(map);
        // map_meta_pub->publish(map.info);
    }
};

int main(int argc, char *argv[]) {
    // Start ROS 2 node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OrderedMapSpline>());
    rclcpp::shutdown();

    return 0;
}
