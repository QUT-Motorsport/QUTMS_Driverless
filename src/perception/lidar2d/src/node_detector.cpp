#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ctime>

using std::placeholders::_1;

class LiDARProcessor : public rclcpp::Node{
   private:
    // Store clock
    rclcpp::Clock::SharedPtr rosclock = this->get_clock();
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstPtr &input)
    {
        clock_t start = clock();

        /* Process the point cloud */
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        /* Creating the KdTree from input point cloud*/
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

        pcl::fromROSMsg(*input, *input_cloud);

        tree->setInputCloud(input_cloud);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(input_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Could not estimate a planar model for the given dataset.");
        }

        std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                            << coefficients->values[1] << " "
                                            << coefficients->values[2] << " " 
                                            << coefficients->values[3] << std::endl;

        std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx: inliers->indices)
        {
            ground_points->points.push_back(input_cloud->points[idx]);
        }

        auto ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*ground_points, *ground_msg);
        ground_msg->header.frame_id = "velodyne";
        //cluster_msg->header.frame_id = cluster->header.frame_id;
        ground_msg->header.stamp = rosclock->now();
        ground_pub->publish(*ground_msg);
        RCLCPP_INFO(this->get_logger(), "time: %f", (clock() - start) / (double)CLOCKS_PER_SEC);
    }

   public:
    LiDARProcessor() : Node("lidar_processor") {
        // Create a ROS subscriber for the input point cloud
        cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 1, std::bind(&LiDARProcessor::cloud_callback, this, _1));

        // Create a ROS publisher for the output point cloud
        ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground", 1);
        RCLCPP_INFO(this->get_logger(),"---Initialised LidarDetector Node---");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto lidar_node = std::make_shared<LiDARProcessor>();
    rclcpp::spin(lidar_node);
    rclcpp::shutdown();

    return 0;
}
