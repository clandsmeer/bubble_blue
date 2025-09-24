/*
File: gravity_compensator v1.0
Author(s): Henry Adam
Description: This file is meant to remove the gravitational vector from the simulated
and real ArduSub IMU.
*/

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

class GravityCompensator : public rclcpp::Node
{
public:
  GravityCompensator()
      : Node("gravity_compensator")
  {
    this->declare_parameter("input_topic", "/mavros/imu/data");
    this->declare_parameter("output_topic", "/mavros/imu/data_comped");
    this->declare_parameter("gravity_strength", 9.80665);

    this->get_parameter("input_topic", input_topic_name_);
    this->get_parameter("output_topic", output_topic_name_);
    this->get_parameter("gravity_strength", gravity_strength_);

    // Create the quality of service profile for the subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth),
        qos_profile);

    // Create the subscriber to the Imu Node
    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        input_topic_name_,
        qos,
        std::bind(
            &GravityCompensator::imu_msg_received_callback, this, std::placeholders::_1));

    // Create the publishers for the acceleration data and orientation data
    compensated_pub = this->create_publisher<sensor_msgs::msg::Imu>(
        output_topic_name_,
        10);
  }

private:
  void imu_msg_received_callback(const sensor_msgs::msg::Imu &msg)
  {
    // create two new messages to send, one Imu and the other pose w/ cov
    sensor_msgs::msg::Imu imu_msg = msg;

    // remove the gravitational acceleration from the accelerometer measurement
    // calculate the gravitational vector in the robot body frame
    tf2::Quaternion orient_inverse(
        -msg.orientation.x,
        -msg.orientation.y,
        -msg.orientation.z,
        msg.orientation.w);

    // apply the inverse orientation to the gravity vector
    tf2::Vector3 gravity_global(0, 0, gravity_strength_);

    tf2::Matrix3x3 inverse_rotation(orient_inverse);

    tf2::Vector3 gravity_body = inverse_rotation * gravity_global;

    // Subtract the body-fixed gravity vector from the imu measurement
    imu_msg.linear_acceleration.x -= gravity_body.x();
    imu_msg.linear_acceleration.y -= gravity_body.y();
    imu_msg.linear_acceleration.z -= gravity_body.z();

    imu_msg.linear_acceleration_covariance[0] = 0.002;
    imu_msg.linear_acceleration_covariance[4] = 0.002;
    imu_msg.linear_acceleration_covariance[8] = 0.002;

    compensated_pub->publish(imu_msg);
  }
  // Declaring the ros2 parameters for the node
  std::string input_topic_name_;
  std::string output_topic_name_;
  double gravity_strength_;

  // Declaring the publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr compensated_pub;
};

// define the main function which actually spins the sensor up
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityCompensator>());
  rclcpp::shutdown();
  return 0;
}
