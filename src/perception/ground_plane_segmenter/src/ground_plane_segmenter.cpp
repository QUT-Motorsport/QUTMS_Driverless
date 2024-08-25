#include "ground_plane_segmenter/ground_plane_segmenter.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace ground_plane_segmenter {

GroundPlaneSegmenterNode::GroundPlaneSegmenterNode(const rclcpp::NodeOptions &options) : Node("ground_plane_segmenter_node", options) {
    pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "velodyne_points", rclcpp::SensorDataQoS(), std::bind(&GroundPlaneSegmenterNode::pcl_callback, this, std::placeholders::_1));
    pcl_ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/ground_plane", rclcpp::SensorDataQoS());
    pcl_objects_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/objects", rclcpp::SensorDataQoS());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setMaxIterations(500);
    crop.setFilterFieldName("z");

    update_params();

    accumulating_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

    RCLCPP_INFO(this->get_logger(), "---Ground Plane Segmenter Node Initialised---");
}

void GroundPlaneSegmenterNode::update_params() {
    // params
    declare_parameter<double>("distance_threshold", 0.01);
    declare_parameter<double>("eps_angle", 5.0);
    declare_parameter<double>("max_z", 0.0);
    declare_parameter<int>("accumulation_count", 1);
    declare_parameter<double>("plane_thresh", 0.01);

    distance_threshold = get_parameter("distance_threshold").as_double();
    eps_angle = get_parameter("eps_angle").as_double();
    max_z = get_parameter("max_z").as_double();
    accumulation_min_count = get_parameter("accumulation_count").as_int();
    plane_thresh = get_parameter("plane_thresh").as_double();

    seg.setDistanceThreshold(distance_threshold);
    seg.setEpsAngle(eps_angle * (M_PI / 180.0f));
    crop.setFilterLimits(-std::numeric_limits<float>::infinity(), max_z);
}

void GroundPlaneSegmenterNode::pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // convert msg to pcl point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);

    // accumulate point clouds
    accumulating_cloud->header = cloud->header;
    *accumulating_cloud += *cloud;

    if (++accumulation_count < accumulation_min_count) {
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    crop.setInputCloud(accumulating_cloud);
    crop.setNegative(false);
    crop.filter(*cropped_cloud);

    // segment the ground plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setInputCloud(cropped_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        return;
    } else {
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cropped_cloud);

        // extract the ground plane
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_plane(new pcl::PointCloud<pcl::PointXYZI>);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*ground_plane);

        // publish the ground plane
        sensor_msgs::msg::PointCloud2::SharedPtr ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*ground_plane, *ground_msg);
        pcl_ground_pub->publish(*ground_msg);

        // extract everything but the ground plane
        pcl::PointCloud<pcl::PointXYZI>::Ptr objects(new pcl::PointCloud<pcl::PointXYZI>);
        extract.setNegative(true);
        extract.filter(*objects);

        // extract everything above the max_z and add it to the objects
        pcl::PointCloud<pcl::PointXYZI>::Ptr other(new pcl::PointCloud<pcl::PointXYZI>);
        crop.setNegative(true);
        crop.filter(*other);
        *objects += *other;

        // filter out any points that are less than plane_thresh from the estimated plane
        const float A = coefficients->values[0];
        const float B = coefficients->values[1];
        const float C = coefficients->values[2];
        const float D = coefficients->values[3];

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_objects(new pcl::PointCloud<pcl::PointXYZI>);
        filtered_objects->header = cloud->header;

        for (auto point : objects->points) {
            float distance = std::abs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C);
            if (distance > plane_thresh) {
                filtered_objects->points.push_back(point);
            }
        }

        // publish the objects
        sensor_msgs::msg::PointCloud2::SharedPtr objects_msg(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*filtered_objects, *objects_msg);
        pcl_objects_pub->publish(*objects_msg);
    }

    // reset the accumuating cloud
    accumulating_cloud->clear();
    accumulation_count = 0;
}

}  // namespace ground_plane_segmenter

RCLCPP_COMPONENTS_REGISTER_NODE(ground_plane_segmenter::GroundPlaneSegmenterNode)
