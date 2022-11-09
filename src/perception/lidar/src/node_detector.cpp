#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
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

class LiDARProcessor : public rclcpp::Node {
   private:
    rclcpp::Clock::SharedPtr rosclock = this->get_clock();

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstPtr& msg) {
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
        scan_cropper.setMin(Eigen::Vector4f(0.0, -7.5, -0.5, 0.0));
        scan_cropper.setMax(Eigen::Vector4f(15.0, 7.5, 0.2, 1.0));
        scan_cropper.filter(*cropped);

        // Crop below "ground level" to segment ground
        pcl::CropBox<pcl::PointXYZI> ground_cropper;
        ground_cropper.setInputCloud(cropped);
        ground_cropper.setMin(Eigen::Vector4f(0.0, -7.5, -0.5, 0.0));
        ground_cropper.setMax(Eigen::Vector4f(15.0, 7.5, -0.2, 1.0));
        ground_cropper.filter(*ground_crop);

        // Crop above "ground level"
        pcl::CropBox<pcl::PointXYZI> non_ground_cropper;
        non_ground_cropper.setInputCloud(cropped);
        non_ground_cropper.setMin(Eigen::Vector4f(0.0, -7.5, -0.19, 0.0));
        non_ground_cropper.setMax(Eigen::Vector4f(15.0, 7.5, 0.2, 1.0));
        non_ground_cropper.filter(*non_ground_crop);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Segment
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.08);  // metres from line
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

        // // Cluster non-ground points into cones
        // // Creating the KdTree object for the search method of the extraction
        // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        // tree->setInputCloud(non_ground);
        // // Euclidean clustering objects
        // std::vector<pcl::PointIndices> cluster_indices;
        // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        // ec.setClusterTolerance(0.6);
        // ec.setMinClusterSize(20);
        // ec.setMaxClusterSize(200);
        // ec.setSearchMethod(tree);
        // ec.setInputCloud(non_ground);
        // ec.extract(cluster_indices);

        // pcl::PointCloud<pcl::PointXYZI>::Ptr clusters(new pcl::PointCloud<pcl::PointXYZI>);
        // int j = 0;
        // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end
        // (); ++it)
        // {
        //     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        //     cloud_cluster->push_back ((*non_ground)[*pit]); //*
        //     cloud_cluster->width = cloud_cluster->size ();
        //     cloud_cluster->height = 1;
        //     cloud_cluster->is_dense = true;

        //     std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." <<
        //     std::endl; j++; *clusters += *cloud_cluster;
        // }

        // pointcloud to vector of tuples
        std::vector<std::tuple<float, float, float>> points;
        for (auto point : non_ground->points) {
            points.push_back(std::make_tuple(point.x, point.y, point.z));
        }
        // DBSCAN clustering
        std::vector<std::vector<size_t>> db_idxs = dbscan(points, 1, 8);

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

   public:
    LiDARProcessor() : Node("lidar_processor") {
        // Create a ROS subscriber for the input point cloud
        cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 1, std::bind(&LiDARProcessor::cloud_callback, this, _1));

        // Create a ROS publisher for the output point cloud
        ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne/ground", 1);
        non_ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne/non_ground", 1);
        cluster_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne/clusters", 1);
        RCLCPP_INFO(this->get_logger(), "---Initialised Lidar Detector Node---");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto lidar_node = std::make_shared<LiDARProcessor>();
    rclcpp::spin(lidar_node);
    rclcpp::shutdown();

    return 0;
}
