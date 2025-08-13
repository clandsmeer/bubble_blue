/*
File: dvl_message_converter.cpp v1.0
Author: Henry Adam
Date: Aug 12, 2025

This file takes in messages from the ros2 dvl package and converts them to twistWithCovarianceStamped
message types. This way, the robot_localization ekf can read them and fuse them into usable data. 
*/

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "dvl_msgs/msg/dvl.hpp"

class ConvertDVL : public rclcpp::Node
{
public:
  ConvertDVL()
  : Node("dvl_message_converter")
  {

    // Define the incoming and outgoing topics
    this->declare_parameter("input_topic", "/dvl/data");
    this->get_parameter("input_topic", input_topic_name_);

    this->declare_parameter("output_topic", "/dvl/twist_data");
    this->get_parameter("output_topic", output_topic_name_);

    dvl_subscription = this->create_subscription<dvl_msgs::msg::DVL>(
        input_topic_name_,
        10,
        std::bind(&ConvertDVL::conversion_callback, this, std::placeholders::_1));

    converted_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        output_topic_name_,
        10);

    RCLCPP_INFO(this->get_logger(), "DVL Msg Conversion Service Started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_name_.c_str());
  }

private:
  void conversion_callback(const dvl_msgs::msg::DVL &msg)
  {
    try { 
      // create the variable that will store the new noisy message
    geometry_msgs::msg::TwistWithCovarianceStamped converted_msg;

    // transfer the header
    converted_msg.header = msg.header; 

    //put in the velcity measurement
    converted_msg.twist.twist.linear = msg.velocity; 

    // add the covariane methods
    converted_msg.twist.covariance[0] = msg.covariance[0];
    converted_msg.twist.covariance[7] = msg.covariance[4];
    converted_msg.twist.covariance[14] = msg.covariance[8];
    
    converted_publisher->publish(converted_msg);
    } catch (const std::exception& e){ 
      RCLCPP_ERROR(this->get_logger(), "Exception somewhre in the conversion process: %s", e.what());
    }
    

  }

  // declare ros pubsub
  rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_subscription;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr converted_publisher;

  // input and output topic variable declaration
  std::string input_topic_name_;
  std::string output_topic_name_;
  
};

// define the main function which actually spins the sensor up
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvertDVL>());
  rclcpp::shutdown();
  return 0;
}
