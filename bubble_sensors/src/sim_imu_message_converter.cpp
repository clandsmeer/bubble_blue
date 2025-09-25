/*
File: sim_imu_message_converter v1.0
Author(s): Henry Adam
Description: This file is meant to remove the gravitational vector from the simulated IMU
in order to make the behavior of the imu match that of the real system. In addition, it splits
the Imu output into two different topics, reflecting the separation of the body-fixed accelerometer
data and kalman-filterred orientation data from the real IMU
*/

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

class SimImuMessageConverter : public rclcpp::Node
{
public:
  SimImuMessageConverter()
  :Node("sim_imu_message_converter")
  {
    this->declare_parameter("input_topic", "/vectornav/Imu_raw");
    this->declare_parameter("output_topic_accel", "/vectornav/Imu_body");
    this->declare_parameter("output_topic_orient", "/vectornav/filtered_orientation");
    this->declare_parameter("gravity_strength", 9.8);

    this->get_parameter("input_topic", input_topic_name_);
    this->get_parameter("output_topic_accel", accel_topic_name_);
    this->get_parameter("output_topic_orient", orient_topic_name_);
    this->get_parameter("gravity_strength", gravity_strength_);

        // Create the quality of service profile for the subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth),
            qos_profile);

        //Create the subscriber to the Imu Node
    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
                                                                    input_topic_name_,
                                                                    qos,
                                                                    std::bind(
      &SimImuMessageConverter::imu_msg_received_callback, this, std::placeholders::_1)
    );

        //Create the publishers for the acceleration data and orientation data
    accel_pub = this->create_publisher<sensor_msgs::msg::Imu>(
                                                            accel_topic_name_,
                                                            10);

    orient_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                                                            orient_topic_name_,
                                                            10);

    ports_publishing_service = this->create_service<std_srvs::srv::SetBool>(
                                                            "/set_reading_status",
                                                            std::bind(
      &SimImuMessageConverter::ports_publishing_callback, this, std::placeholders::_1,
      std::placeholders::_2));

    ports_publishing = false;

    RCLCPP_INFO(this->get_logger(), "Sim IMU Message Converter Has Started.");
  }

private:
  void imu_msg_received_callback(const sensor_msgs::msg::Imu & msg)
  {
    if (ports_publishing) {
          //create two new messages to send, one Imu and the other pose w/ cov
      sensor_msgs::msg::Imu imu_msg = msg;
      geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;

        //remove the gravitational acceleration from the accelerometer measurement
        //calculate the gravitational vector in the robot body frame
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
      imu_msg.linear_acceleration.x += gravity_body.x();
      imu_msg.linear_acceleration.y += gravity_body.y();
      imu_msg.linear_acceleration.z += gravity_body.z();

      imu_msg.orientation_covariance[0] = 0.0005;
      imu_msg.orientation_covariance[4] = 0.0005;
      imu_msg.orientation_covariance[8] = 0.0005;

        //Now separating out into the two message types:
      pose_msg.header.stamp = msg.header.stamp;
      pose_msg.header.frame_id = msg.header.frame_id;

      pose_msg.pose.pose.orientation = msg.orientation;
      pose_msg.pose.covariance[21] = 0.0005;
      pose_msg.pose.covariance[28] = 0.0005;
      pose_msg.pose.covariance[35] = 0.0005;

      accel_pub->publish(imu_msg);
      orient_pub->publish(pose_msg);
    }

  }

  void ports_publishing_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
                                  // set the ports_publishing parameter to the
    ports_publishing = request->data;

                                 // send successful response
    response->success = true;
    response->message = std::string("Updated Publishing Data to ") +
      (ports_publishing ? "true" : "false");
  }

    //Declaring the ros2 parameters for the node
  std::string input_topic_name_;
  std::string accel_topic_name_;
  std::string orient_topic_name_;
  double gravity_strength_;

    //Declaring the publishers and subscribers
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr accel_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr orient_pub;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ports_publishing_service;
  bool ports_publishing;
};

// define the main function which actually spins the sensor up
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimImuMessageConverter>());
  rclcpp::shutdown();
  return 0;
}
