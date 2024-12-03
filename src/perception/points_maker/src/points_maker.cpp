#include "points_maker/points_maker.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace points_maker {

PointsMakerNode::PointsMakerNode(const rclcpp::NodeOptions &options)
    : Node("points_maker_node", options) {
    ground_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/ground_plane_old", rclcpp::SensorDataQoS(),
        std::bind(&PointsMakerNode::ground_callback, this, std::placeholders::_1));
    objects_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/objects_old", rclcpp::SensorDataQoS(),
        std::bind(&PointsMakerNode::objects_callback, this, std::placeholders::_1));
    velo_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", rclcpp::SensorDataQoS());

    ground_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    objects_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    velo_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    RCLCPP_INFO(this->get_logger(), "---Ground Plane Segmenter Node Initialised---");
}

void PointsMakerNode::ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // convert msg to pcl point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // accumulate point clouds
    *ground_cloud = *cloud;
}

void PointsMakerNode::objects_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // convert msg to pcl point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    *objects_cloud = *cloud;

    // accumulate point clouds
    velo_cloud->header = cloud->header;
    *velo_cloud = *ground_cloud + *objects_cloud;

    // publish the objects
    sensor_msgs::msg::PointCloud2::SharedPtr objects_msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*velo_cloud, *objects_msg);
    velo_pub->publish(*objects_msg);

    // reset the accumuating cloud
    velo_cloud->clear();
}
}  // namespace points_maker

RCLCPP_COMPONENTS_REGISTER_NODE(points_maker::PointsMakerNode)
