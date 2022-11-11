#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ctime>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include "../PCL_DBSCAN/dbscan.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

const int min_cluster_size = 10;
const int max_cluster_size = 500;
const float min_cluster_distance = 1.0;

const float min_x = 0.0;
const float max_x = 15.0;
const float min_y = -6.0;
const float max_y = 6.0;
const float min_z = -0.5;
const float max_z = 0.2;
const float max_z_ground = -0.2;
const float min_z_nground = max_z_ground + 0.01;

const float max_plane_dist = 0.08;

class LiDARProcessor : public rclcpp::Node {
   private:
    rclcpp::Clock::SharedPtr rosclock = this->get_clock();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub;

   public:
    LiDARProcessor() : Node("lidar_processor") {
        // Create a ROS subscriber for the input point cloud
        cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10, std::bind(&LiDARProcessor::cloud_callback, this, _1));

        // Create a ROS publisher for the output point cloud
        ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne/ground", 1);
        non_ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne/non_ground", 1);
        cluster_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne/clusters", 1);
        RCLCPP_INFO(this->get_logger(), "---Initialised Lidar Detector Node---");
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        clock_t start = clock();
        // Process the point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        // Create the filtering object
        pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_crop(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_crop(new pcl::PointCloud<pcl::PointXYZI>);

        // Crop window of interest
        pcl::CropBox<pcl::PointXYZI> scan_cropper;
        scan_cropper.setInputCloud(cloud);
        scan_cropper.setMin(Eigen::Vector4f(min_x, min_y, min_z, 0.0));
        scan_cropper.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
        scan_cropper.filter(*cropped);

        // Crop below "ground level" to segment ground
        pcl::CropBox<pcl::PointXYZI> ground_cropper;
        ground_cropper.setInputCloud(cropped);
        ground_cropper.setMin(Eigen::Vector4f(min_x, min_y, min_z, 0.0));
        ground_cropper.setMax(Eigen::Vector4f(max_x, max_y, max_z_ground, 1.0));
        ground_cropper.filter(*ground_crop);

        // Crop above "ground level"
        pcl::CropBox<pcl::PointXYZI> non_ground_cropper;
        non_ground_cropper.setInputCloud(cropped);
        non_ground_cropper.setMin(Eigen::Vector4f(min_x, min_y, min_z_nground, 0.0));
        non_ground_cropper.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
        non_ground_cropper.filter(*non_ground_crop);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Segment
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(max_plane_dist);  // metres from line
        seg.setMaxIterations(10);

        seg.setInputCloud(ground_crop);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            RCLCPP_INFO(this->get_logger(), "Could not estimate a planar model for this scan.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_seg(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_seg(new pcl::PointCloud<pcl::PointXYZI>);
        // extract ground points and non-ground points
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointIndices::Ptr filtered_idxs(new pcl::PointIndices());
        extract.setInputCloud(ground_crop);
        extract.setIndices(inliers);
        // ground seg
        extract.setNegative(false);
        extract.filter(*ground_seg);
        // non-ground
        extract.setNegative(true);
        extract.filter(*non_ground_seg);

        // Add non-ground crop and non-ground seg
        pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZI>);
        *non_ground = *non_ground_seg + *non_ground_crop;

        // pointcloud to vector of tuples
        std::vector<std::tuple<float, float, float>> points;
        for (auto point : non_ground->points) {
            points.push_back(std::make_tuple(point.x, point.y, point.z));
        }
        // DBSCAN clustering
        std::vector<std::vector<size_t>> db_idxs =
            dbscan(points, min_cluster_distance, min_cluster_size, max_cluster_size);

        // Create clusters
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusters(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto idxs : db_idxs) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
            for (auto idx : idxs) {
                cloud_cluster->push_back((*non_ground)[idx]);
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points."
                      << std::endl;
            *clusters += *cloud_cluster;
        }

        // Publish the ground point cloud
        auto ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*ground_seg, *ground_msg);
        ground_msg->header = msg->header;
        ground_pub->publish(*ground_msg);
        // Publish the non-ground point cloud
        auto non_ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*non_ground, *non_ground_msg);
        non_ground_msg->header = msg->header;
        non_ground_pub->publish(*non_ground_msg);
        // Publish the clusters point cloud
        auto cluster_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*clusters, *cluster_msg);
        cluster_msg->header = msg->header;
        cluster_pub->publish(*cluster_msg);

        RCLCPP_INFO(this->get_logger(), "time: %f", (clock() - start) / (double)CLOCKS_PER_SEC);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto lidar_node = std::make_shared<LiDARProcessor>();
    rclcpp::spin(lidar_node);
    rclcpp::shutdown();

    return 0;
}
