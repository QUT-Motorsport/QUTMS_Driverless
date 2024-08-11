#ifndef GROUND_PLANE_SEGMENTER__GROUND_PLANE_SEGMENTER_HPP_
#define GROUND_PLANE_SEGMENTER__GROUND_PLANE_SEGMENTER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/qos.hpp"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/passthrough.h"

namespace ground_plane_segmenter {

class GroundPlaneSegmenterNode : public rclcpp::Node {
   public:
    GroundPlaneSegmenterNode(const rclcpp::NodeOptions& options);

   private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_ground_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_objects_pub;

    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void update_params();

    // params
    double distance_threshold;
    double eps_angle;
    double max_z;
    int accumulation_min_count;
    double plane_thresh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulating_cloud;
    int accumulation_count = 0;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PassThrough<pcl::PointXYZ> crop;
};

}  // namespace ground_plane_segmenter

#endif  // GROUND_PLANE_SEGMENTER__GROUND_PLANE_SEGMENTER_HPP_