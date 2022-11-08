#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstPtr &msg)
    {
        clock_t start = clock();
        // Process the point cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        // Create the filtering object
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::CropBox<pcl::PointXYZI> scan_filter;
        scan_filter.setInputCloud(cloud);
        scan_filter.setMin(Eigen::Vector4f(0.0, -7.5, -0.5, 0.0));
        scan_filter.setMax(Eigen::Vector4f(20.0, 7.5, 0.2, 1.0));
        scan_filter.filter(*cloud_filtered);

        auto filtered_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud_filtered, *filtered_msg);
        filtered_msg->header = msg->header;
        filtered_pub->publish(*filtered_msg);

        // RCLCPP_INFO(this->get_logger(), "time: %f", (clock() - start) / (double)CLOCKS_PER_SEC);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.02);
        seg.setAxis(Eigen::Vector3f(0, 0, 1));

        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Could not estimate a planar model for this scan.");
            return;
        }

        // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
        //                                     << coefficients->values[1] << " "
        //                                     << coefficients->values[2] << " " 
        //                                     << coefficients->values[3] << std::endl;
        // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_points(new pcl::PointCloud<pcl::PointXYZI>);
        // extract ground points and non-ground points
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        pcl::PointIndices::Ptr filtered_idxs (new pcl::PointIndices ());
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        // ground
        extract.setNegative(false);
        extract.filter(*ground_points);
        // non-ground
        extract.setNegative(true);
        extract.filter(*non_ground_points);

        auto ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*ground_points, *ground_msg);
        ground_msg->header = msg->header;
        ground_pub->publish(*ground_msg);

        auto non_ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*non_ground_points, *non_ground_msg);
        non_ground_msg->header = msg->header;
        non_ground_pub->publish(*non_ground_msg);

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
        filtered_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne/filtered", 1);
        RCLCPP_INFO(this->get_logger(),"---Initialised Lidar Detector Node---");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto lidar_node = std::make_shared<LiDARProcessor>();
    rclcpp::spin(lidar_node);
    rclcpp::shutdown();

    return 0;
}
