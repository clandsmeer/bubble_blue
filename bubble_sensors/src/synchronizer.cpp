/*
File: gravity_compensator v1.0
Author(s): Henry Adam
Description: This file is meant to remove the gravitational vector from the simulated IMU
in order to make the behavior of the imu match that of the real system. In addition, it splits
the Imu output into two different topics, reflecting the separation of the body-fixed accelerometer
data and kalman-filterred orientation data from the real IMU
*/

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class Synchronizer : public rclcpp::Node
{
public:
  //constructor
  Synchronizer()
  : Node("synchronizer")
  {
    this->declare_parameter("publishing_frequency", 30.0);
    this->declare_parameter("imu_accels_input_topic", "/vectornav/Imu_body");
    this->declare_parameter("imu_orient_input_topic", "/vectornav/filtered_orientation");
    this->declare_parameter("dvl_input_topic", "/dvl/twist_data");

    this->get_parameter("publishing_frequency", publishing_frequency_);
    this->get_parameter("imu_accels_input_topic", imu_accels_input_topic_);
    this->get_parameter("imu_orient_input_topic", imu_orient_input_topic_);
    this->get_parameter("dvl_input_topic", dvl_input_topic_);

    // Create the quality of service profile for the subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth),
            qos_profile);

    imu_accels_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_accels_input_topic_,
      qos,
      std::bind(&Synchronizer::imu_accel_sub_callback, this, std::placeholders::_1)
    );

    imu_orient_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      imu_orient_input_topic_,
      qos,
      std::bind(&Synchronizer::imu_orient_sub_callback, this, std::placeholders::_1)
    );

    dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      dvl_input_topic_,
      qos,
      std::bind(&Synchronizer::dvl_sub_callback, this, std::placeholders::_1)
    );

    std::chrono::duration<double> period = std::chrono::duration<double>(1.0 /
      publishing_frequency_);
    // auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    timer_ = this->create_timer(
      period,
      std::bind(&Synchronizer::synchronizer_callback, this)
    );

    imu_orient_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      imu_orient_input_topic_ + "_synced",
      10
    );

    dvl_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      dvl_input_topic_ + "_synced",
      10
    );

    imu_accels_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      imu_accels_input_topic_ + "_synced",
      10
    );
  }

private:
  void imu_accel_sub_callback(const sensor_msgs::msg::Imu & accel_msg)
  {
    latest_imu_accel_msg_ = accel_msg;
  }

  void imu_orient_sub_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & orient_msg)
  {
    latest_imu_orient_msg_ = orient_msg;
  }

  void dvl_sub_callback(const geometry_msgs::msg::TwistWithCovarianceStamped & dvl_msg)
  {
    latest_dvl_msg_ = dvl_msg;
  }

  void synchronizer_callback()
  {
    rclcpp::Time synced_time = this->now();

      // Update header.stamp for each message
    latest_imu_accel_msg_.header.stamp = synced_time;
    latest_imu_orient_msg_.header.stamp = synced_time;
    latest_dvl_msg_.header.stamp = synced_time;

      // Publish the synchronized messages
    imu_accels_pub_->publish(latest_imu_accel_msg_);
    imu_orient_pub_->publish(latest_imu_orient_msg_);
    dvl_pub_->publish(latest_dvl_msg_);
  }

  double publishing_frequency_;
  std::string imu_accels_input_topic_;
  std::string imu_orient_input_topic_;
  std::string dvl_input_topic_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_accels_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr imu_orient_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_accels_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr imu_orient_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Imu latest_imu_accel_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_imu_orient_msg_;
  geometry_msgs::msg::TwistWithCovarianceStamped latest_dvl_msg_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Synchronizer>());
  rclcpp::shutdown();
  return 0;
}
