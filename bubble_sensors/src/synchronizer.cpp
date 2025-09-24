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
    this->declare_parameter("ardusub_imu_input_topic", "/mavros/imu/data_comped");
    this->declare_parameter("max_measurement_delay_sec", 0.5);

    this->get_parameter("publishing_frequency", publishing_frequency_);
    this->get_parameter("imu_accels_input_topic", imu_accels_input_topic_);
    this->get_parameter("imu_orient_input_topic", imu_orient_input_topic_);
    this->get_parameter("dvl_input_topic", dvl_input_topic_);
    this->get_parameter("ardusub_imu_input_topic", ardusub_imu_input_topic_);
    this->get_parameter("max_measurement_delay_sec", max_measurement_delay_sec_);

    // Create the quality of service profile for the subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(
            qos_profile.history,
            qos_profile.depth),
            qos_profile);

    //subscription for the Vectornav IMU
    imu_accels_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_accels_input_topic_,
      qos,
      std::bind(&Synchronizer::imu_accel_sub_callback, this, std::placeholders::_1)
    );

    //subscription for the Vevtornav filtered orientation output
    imu_orient_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      imu_orient_input_topic_,
      qos,
      std::bind(&Synchronizer::imu_orient_sub_callback, this, std::placeholders::_1)
    );

    //subscription for the DVL
    dvl_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      dvl_input_topic_,
      qos,
      std::bind(&Synchronizer::dvl_sub_callback, this, std::placeholders::_1)
    );

    // subscription for the ArduSub IMU
    ardusub_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      ardusub_imu_input_topic_,
      qos,
      std::bind(&Synchronizer::ardusub_imu_sub_callback, this, std::placeholders::_1)
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

    ardusub_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      ardusub_imu_input_topic_ + "_synced",
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

  void ardusub_imu_sub_callback(const sensor_msgs::msg::Imu & ardusub_imu_msg)
  {
    latest_ardusub_imu_msg_ = ardusub_imu_msg;
  }

  void synchronizer_callback()
  {
    rclcpp::Time synced_time = this->now();

      // For each message type, only publish once the message if the header has been updated.
      // This ensures that it does not publish zeros before sensor reading have come in
    if (latest_imu_accel_msg_.header.stamp.sec != 0 ||
      latest_imu_accel_msg_.header.stamp.nanosec != 0)
    {
      // now enforce the constraint on message delay
      rclcpp::Time msg_time(latest_imu_accel_msg_.header.stamp);
      double time_delay = synced_time.seconds() - msg_time.seconds();
      if (time_delay <= max_measurement_delay_sec_) {
        latest_imu_accel_msg_.header.stamp = synced_time;
        imu_accels_pub_->publish(latest_imu_accel_msg_);
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "VN100 Acceleration Measurement is Too Old For Synchronizer! Is it still publishing?");
      }
    }

    if (latest_imu_orient_msg_.header.stamp.sec != 0 ||
      latest_imu_orient_msg_.header.stamp.nanosec != 0)
    {
      rclcpp::Time msg_time(latest_imu_orient_msg_.header.stamp);
      double time_delay = synced_time.seconds() - msg_time.seconds();
      if (time_delay <= max_measurement_delay_sec_) {
        latest_imu_orient_msg_.header.stamp = synced_time;
        imu_orient_pub_->publish(latest_imu_orient_msg_);
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "VN100 Orientation is Too Old For Synchronizer! Is it still publishing?");
      }
    }

    if (latest_dvl_msg_.header.stamp.sec != 0 || latest_dvl_msg_.header.stamp.nanosec != 0) {
      rclcpp::Time msg_time(latest_dvl_msg_.header.stamp);
      double time_delay = synced_time.seconds() - msg_time.seconds();
      if (time_delay <= max_measurement_delay_sec_) {
        latest_dvl_msg_.header.stamp = synced_time;
        dvl_pub_->publish(latest_dvl_msg_);
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "VN100 Measurement is Too Old For Synchronizer! Is it still publishing?");
      }
    }

    if (latest_ardusub_imu_msg_.header.stamp.sec != 0 ||
      latest_ardusub_imu_msg_.header.stamp.nanosec != 0)
    {
      rclcpp::Time msg_time(latest_ardusub_imu_msg_.header.stamp);
      double time_delay = synced_time.seconds() - msg_time.seconds();
      if (time_delay <= max_measurement_delay_sec_) {
        latest_ardusub_imu_msg_.header.stamp = synced_time;
        ardusub_imu_pub_->publish(latest_ardusub_imu_msg_);
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "VN100 Measurement is Too Old For Synchronizer! Is it still publishing?");
      }
    }

  }

  double publishing_frequency_;
  std::string imu_accels_input_topic_;
  std::string imu_orient_input_topic_;
  std::string dvl_input_topic_;
  std::string ardusub_imu_input_topic_;
  double max_measurement_delay_sec_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_accels_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr imu_orient_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ardusub_imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_accels_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr imu_orient_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr dvl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ardusub_imu_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Imu latest_imu_accel_msg_;
  geometry_msgs::msg::PoseWithCovarianceStamped latest_imu_orient_msg_;
  geometry_msgs::msg::TwistWithCovarianceStamped latest_dvl_msg_;
  sensor_msgs::msg::Imu latest_ardusub_imu_msg_;

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Synchronizer>());
  rclcpp::shutdown();
  return 0;
}
