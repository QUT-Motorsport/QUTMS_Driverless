#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <vector>

#include "rclcpp/rclcpp.hpp"
// include messages from ROS sensors
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "fs_msgs/msg/control_command.hpp"
// include messages needed for node communication
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

double *xp;
double *yp;

class MovementControl : public rclcpp::Node{ // create class with inheritance from ROS node
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
            // math_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            //     "math_output", 10, std::bind(&MovementControl::move_callback, this, _1));
            math_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
                "math_output", 10, std::bind(&MovementControl::move_callback, this, _1));

            // publish to the "/fsds/control_command" node as a 'ControlCommand' FS message
            control_publisher_ = this->create_publisher<fs_msgs::msg::ControlCommand>(
                "/fsds/control_command", 10);
        }

    private:

    // image_callback to test if camera is running
    void image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) const{
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

    // geo_callback to receive subscribed geometry messages
    void geo_callback(const geometry_msgs::msg::TwistStamped::SharedPtr geo_msg) const{
        // retrieve x and y velocities
        double vel_x = geo_msg->twist.linear.x; 
        double vel_y = geo_msg->twist.linear.y;

        xp = &vel_x;
        yp = &vel_y;

    }

    // move_callback to publish movement commands
    // void move_callback(const std_msgs::msg::Float32MultiArray::SharedPtr math_msg) const{
    void move_callback(const std_msgs::msg::Float32::SharedPtr math_msg) const{

        // std::vector<float> cones = math_msg->data;
        // int length = sizeof(cones)/sizeof(cones[0]);

        float average_y = math_msg->data;
        
        RCLCPP_INFO(this->get_logger(), "Heard avg: '%lf'", average_y);

        double max_steering = 0.3;
        double calc_steering = 0.0;

        double max_throttle = 0.2; // m/s^2
        int target_vel = 4; // m/s
        double calc_throttle = 0.0;

        double vel_x = *xp;
        double vel_y = *yp;

        RCLCPP_INFO(this->get_logger(), "X vel: %lf, Y vel: %lf", vel_x, vel_y);

        // calculate throttle
        // velocity in the vehicle's frame
        double vel = sqrt(vel_x*vel_x + vel_y*vel_y);
        // the lower the velocity, the more throttle, up to max_throttle
        double p_vel = (1 - (vel / target_vel));
        if ( p_vel > 0 ){
            calc_throttle = max_throttle * p_vel;
        }
        else if ( p_vel <= 0 ){
            calc_throttle = 0.0;
        }

        // determine steering
        // if there are more cones on the left, go to the left, else go to the right.
        // float average_y = 0;
        // for(int i=0; i<length; i++){
        //     average_y += cones[i][1]; // y data in 2nd element of each array
        // }
        // average_y = average_y / length; // find average

        if( average_y > 0 ){
            calc_steering = -max_steering;
        }
        else if( average_y == 0 ){
            calc_steering = 0;
        }
        else if( average_y < 0 ){
            calc_steering = max_steering;
        }

        auto message = fs_msgs::msg::ControlCommand();
        message.throttle = calc_throttle;
        message.steering = calc_steering;
        message.brake = 0;
        
        control_publisher_->publish(message);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr math_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr math_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr geo_subscriber_;
    rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_publisher_;

};

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementControl>());
    rclcpp::shutdown();
    return 0;
}
