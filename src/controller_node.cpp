#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
namespace controller_node{
    class ControllerNode : public rclcpp::Node{
        private:
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ajoy_sub_;
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
            rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr actuators_pub_;
            std::string joy_frame_id_;
        public:
        ControllerNode():
        Node("controller_node")
        {
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),std::bind(&ControllerNode::joyCallback, this, std::placeholders::_1));
            twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("joystick_params",1);
            actuators_pub_ = this->create_publisher<std_msgs::msg::Int32>("actuators_params",1);
            joy_frame_id_ = "normal_mode";
        }
        

        private:

        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg){
            // if push a button, change the frame_id to aiming_mode
            if(msg->buttons[1] == 1){
                joy_frame_id_ = "normal_mode";
            }
            //if push b button, change the frame_id to normal_mode
            if(msg->buttons[2] == 1){
                joy_frame_id_ = "aiming_mode";
            }
            
            //publish undercarrige message
            // the right joystick is geometry_msgs::Twist.linear.x, geometry_msgs::Twist.linear.y
            // the left joystick is geometry_msgs::Twist.angular.x, geometry_msgs::Twist.angular.y
            geometry_msgs::msg::TwistStamped twiststmped;
            twiststmped.header.frame_id = joy_frame_id_;
            twiststmped.twist.linear.x = msg->axes[0];
            twiststmped.twist.linear.y = msg->axes[1];
            twiststmped.twist.angular.x= msg->axes[2];
            twiststmped.twist.angular.y= msg->axes[3];
            twist_pub_->publish(twiststmped);

            //publish button message
            //TODO


            //actuators_pub_.publish(something);

        }
    };

} // namespace controller_node

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controller_node::ControllerNode>());
    rclcpp::shutdown();
    return 0;
}