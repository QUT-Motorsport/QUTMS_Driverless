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
#include "qutms_msgs/msg/cone_data.hpp"
#include "qutms_msgs/msg/location.hpp"
// include messages needed for node communication
#include "std_msgs/msg/float32_multi_array.hpp"

using std::placeholders::_1;

class MovementControl : public rclcpp::Node{ // create class with inheritance from ROS node
    
    public:
        MovementControl() : Node("control_node"){ // name of our node, constructed from ROS
            // subscribe to the "/fsds/camera/cam1" node as an 'Image' sensor message
            // bind subscribed messages to function 'image_callback'
            camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/fsds/camera/cam1", 10, std::bind(&MovementControl::image_callback, this, _1));

            // subscribe to the "/fsds/gss" node as a 'TwistStamped' geometry message
            // bind subscribed messages to function 'geo_callback'
            // geo_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            //     "/fsds/gss", 10, std::bind(&MovementControl::geo_callback, this, _1));

            // subscribe to the "math_output" custom node as a 'ConeData' qutms_msgs message
            // bind subscribed messages to function 'move_callback'
            math_subscription_ = this->create_subscription<qutms_msgs::msg::ConeData>(
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

        // move_callback to publish movement commands
        void move_callback(const qutms_msgs::msg::ConeData::SharedPtr math_msg) const{

            float len = math_msg->array_len;
            float vel = math_msg->car_vel;
            // std::vector<float> cones = math_msg->cone_array; // BANE OF MY EXISTENCE THIS THING, CANT GET IT TO WORK!!!

            RCLCPP_INFO(this->get_logger(), "Heard len: '%lf'", len);
            RCLCPP_INFO(this->get_logger(), "Heard vel: '%lf'", vel);

            float steering_p = 5; // steering proportion
            float calc_steering = 0.0; // initial steering
            float average_y = 0.0; // initial cones

            float max_throttle = 0.2; // m/s^2
            int target_vel = 4; // m/s
            float calc_throttle = 0.0; // initial throttle


            // calculate throttle from velocity in the vehicle's frame
            // the lower the velocity, the more throttle, up to max_throttle
            float p_vel = (1 - (vel / target_vel));
            if ( p_vel > 0 ){
                calc_throttle = max_throttle * p_vel;
            }
            else if ( p_vel <= 0 ){
                calc_throttle = 0.0;
            }

            // calculate steering from relative cone location
            // if ( len != 0 ){
            //     for ( int i=0; i<len; i++ ){
            //         average_y += cones[i][1];
            //     average_y = average_y / len;
            //     }
            // }
            // else{
            //     average_y = 0;
            // }

            calc_steering = -steering_p * average_y;
            if ( calc_steering > 1.0 ){
                calc_steering = 1.0;
            }
            else if ( calc_steering < -1.0 ){
                calc_steering = -1.0;
            }

            auto control = fs_msgs::msg::ControlCommand();
            control.throttle = calc_throttle;
            control.steering = calc_steering;
            control.brake = 0;
            
            control_publisher_->publish(control);
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        rclcpp::Subscription<qutms_msgs::msg::ConeData>::SharedPtr math_subscription_;
        // rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr geo_subscription_;
        rclcpp::Publisher<fs_msgs::msg::ControlCommand>::SharedPtr control_publisher_;
};

int main(int argc, char * argv[]){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementControl>());
    rclcpp::shutdown();
    return 0;
}
