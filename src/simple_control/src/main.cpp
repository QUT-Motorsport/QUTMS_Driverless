#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "fs_msgs/msg/control_command.hpp"

using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node
{
    public:
    ImageSubscriber()
    : Node("minimal_subscriber")
    {
        camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fsds/camera/cam1", 10, std::bind(&ImageSubscriber::image_callback, this, _1)
        );
        control_publisher_ = this->create_publisher<fs_msgs::msg::ControlCommand>("/fsds/control_command", 10);
    }

    private:
    // Callback
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("Camera View", cv_ptr->image);
        cv::waitKey(1);

        auto message = fs_msgs::msg::ControlCommand();
        message.throttle = 0.5;
        message.steering = -1;
        message.brake = 0;
        control_publisher_->publish(message);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_publisher_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
