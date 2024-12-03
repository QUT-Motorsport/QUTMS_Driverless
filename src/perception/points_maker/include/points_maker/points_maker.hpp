#ifndef POINTS_MAKER__POINTS_MAKER_HPP_
#define POINTS_MAKER__POINTS_MAKER_HPP_

#include <memory>

#include "pcl/filters/extract_indices.h"
#include "pcl/filters/passthrough.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace points_maker {

class PointsMakerNode : public rclcpp::Node {
   public:
    PointsMakerNode(const rclcpp::NodeOptions& options);

   private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr objects_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velo_pub;

    void ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void objects_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr objects_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr velo_cloud;
};

}  // namespace points_maker

#endif  // POINTS_MAKER__POINTS_MAKER_HPP_
