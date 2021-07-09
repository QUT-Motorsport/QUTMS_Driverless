#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
// include messages from ROS sensors
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "fs_msgs/msg/control_command.hpp"
// include messages needed for node communication
#include "std_msgs/msg/float32_multi_array.hpp"

using std::placeholders::_1;

class MovementControl : public rclcpp::Node // create class with inheritance from ROS node
{
    public:
    MovementControl()
    : Node("control_node"){ // name of our node, constructed from ROS
        // subscribe to the "/fsds/camera/cam1" node as an 'Image' sensor message
        // bind subscribed messages to function 'image_callback'
        camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/fsds/camera/cam1", 10, std::bind(&MovementControl::image_callback, this, _1));

        // subscribe to the "/fsds/gss" node as a 'TwistStamped' geometry message
        // bind subscribed messages to function 'geo_callback'
        geo_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/fsds/gss", 10, std::bind(&MovementControl::geo_callback, this, _1));

        // subscribe to the "math_output" custom node as a 'Float32MultiArray' standard message
        // bind subscribed messages to function 'move_callback'
        math_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "math_output", 10, std::bind(&MovementControl::move_callback, this, _1));

        // publish to the "/fsds/control_command" node as a 'ControlCommand' FS message
        control_publisher_ = this->create_publisher<fs_msgs::msg::ControlCommand>(
            "/fsds/control_command", 10);
    }

    private:
    // image_callback to test if camera is running
    void image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) const
    {
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::imshow("Camera View", cv_ptr->image);
        cv::waitKey(1);
    }

    // geo_callback 
    void geo_callback(const geometry_msgs::msg::TwistStamped::SharedPtr geo_msg) const
    {
        // retrieve x and y velocities
        float vel_x = geo_msg->twist.linear.x; 
        float vel_x = geo_msg->twist.linear.y;

    }

    void move_callback(const std_msgs::msg::Float32MultiArray::SharedPtr math_msg) const{
        auto cones[] = {math_msg->data};

        int length = sizeof(cones);
        float max_steering = 0.3;
        float average_y = 0.0;
        float calc_steering = 0.0;

        float max_throttle = 0.2; // m/s^2
        int target_vel = 4; // m/s
        float calc_throttle = 0.0;

        if ( sizeof(cones) != 0 ){
            // calculate throttle
            // velocity in the vehicle's frame
            float vel = sqrt(vel_x*vel_x + vel_y*vel_y);
            // the lower the velocity, the more throttle, up to max_throttle
            float p_vel = (1 - (vel / target_vel));
            if ( p_vel > 0 ){
                calc_throttle = max_throttle * p_vel;
            }
            else if ( p_vel <= 0 ){
                calc_throttle = 0.0;
            }

            // determine steering
            // if there are more cones on the left, go to the left, else go to the right.
            float max_steering = 0.3;
            float average_y = 0;
            for(int i=0; i<length; i++){
                average_y += cones[i][1]; // y data in 2nd element of each array
            }
            average_y = average_y / length; // find average

            if( average_y > 0 ){
                calc_steering = -max_steering;
            }
            else{
                calc_steering = max_steering;
            }

            auto message = fs_msgs::msg::ControlCommand();
            message.throttle = calc_throttle;
            message.steering = calc_steering;
            message.brake = 0;
            
            control_publisher_->publish(message);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr math_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr geo_subscriber_;
    rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_publisher_;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementControl>());
    rclcpp::shutdown();
    return 0;
}
