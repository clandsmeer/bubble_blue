/*
vectornav_dvl.cpp V1.0
Author: Henry Adam
Purpose:
This file is meant to act as the node representing the vectornav IMU
for the Blue simulation used by Bubble Robotics. Specific parameters will be updatable via
*/

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <random>
#include <math.h>

class VectornavIMU : public rclcpp::Node
{
public:
  VectornavIMU()
      : Node("vectornav_imu")
  {
    // create/read the parameters for the sensor in the constructor:
    //  standard deviation
    this->declare_parameter<double>("accel_noise_density_mg", 0.14);
    this->get_parameter("accel_noise_density_mg", accel_noise_density_);

    this->declare_parameter<double>("gyro_noise_density_deg", 0.0035);
    this->get_parameter("gyro_noise_density_deg", gyro_noise_density_);

    this->declare_parameter<int>("sample_rate", 400);
    this->get_parameter("sample_rate", sample_rate_);

    // defining which measurements will be noisy
    this->declare_parameter<double>("accel_bias_stability_mghr", 0.04);
    this->get_parameter("accel_bias_stability_mghr", accel_bias_stability_);

    this->declare_parameter<double>("gyro_bias_stability_deghr", 10);
    this->get_parameter("gyro_bias_stability_deghr", gyro_bias_stability_);

    this->declare_parameter<double>("resolution", 0.00001);
    this->get_parameter("resolution", resolution_);

    // Define the incoming and outgoing topics
    this->declare_parameter("input_topic", "/model/bluerov2/odometry");
    this->get_parameter("input_topic", input_topic_name_);

    this->declare_parameter("output_topic", "/vn100/imu");
    this->get_parameter("output_topic", output_topic_name_);

    odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        input_topic_name_,
        10,
        std::bind(&VectornavIMU::odom_callback, this, std::placeholders::_1));

    measurement_publisher = this->create_publisher<sensor_msgs::msg::Imu>(
        output_topic_name_,
        10);

    // Declare the covariance
    first_message_ = true;

    // IMU sensor setup
    //  converting noise density to noise:
    accel_stddev_ = (accel_noise_density_ * 9.81 / 1000) * std::sqrt(sample_rate_);
    gyro_stddev_ = (gyro_noise_density_ * M_PI / 180) * std::sqrt(sample_rate_);

    // covariances for acceleration and gyro
    accel_cov_ = accel_stddev_ * accel_stddev_ + (accel_bias_stability_ * 9.81 / 1000) *
                                                     (accel_bias_stability_ * 9.81 / 1000);
    gyro_cov_ = gyro_stddev_ * gyro_stddev_ + (gyro_bias_stability_ * M_PI / 180 / 3600) *
                                                  (gyro_bias_stability_ * M_PI / 180 / 3600);

    // bias initialization
    // accel_bias_.x = accel_bias_.y = accel_bias_.z = 0;
    // gyro_bias_.x = gyro_bias_.y = gyro_bias_.z = 0;

    // seed the random generator for max bias drift:
    std::srand(std::time(0)); // seed w/ start time of program

    RCLCPP_INFO(this->get_logger(), "Vectornav IMU Started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_name_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry &msg)
  {
    // create the variable that will store the new noisy message
    sensor_msgs::msg::Imu noisy_msg;

    // first add the header
    noisy_msg.header = msg.header;

    // gaussians for gyro and accelerometer:
    std::normal_distribution diff_accel_(0.0, accel_stddev_);
    std::normal_distribution diff_gyro_(0.0, gyro_stddev_);

    // do the velocity differentiation to get instantaneous acceleration
    // note that there will be numerical noise
    if (first_message_)
    {
      last_velocity_ = msg.twist.twist.linear;
      last_time_ = rclcpp::Time(msg.header.stamp);
      first_message_ = false;
      return; // Skip first message since we need previous data
    }

    rclcpp::Time current_time = rclcpp::Time(msg.header.stamp);
    double dt = (current_time - last_time_).seconds();

    if (dt <= 0 || dt >= 1)
    {
      last_velocity_ = msg.twist.twist.linear;
      last_time_ = current_time;
      RCLCPP_WARN(this->get_logger(), "Invalid dt for acceleration derivation: %f", dt);
      return;
    }

    geometry_msgs::msg::Vector3 curr_velocity = msg.twist.twist.linear;
    geometry_msgs::msg::Vector3 accel;

    // update the total bias with random walk drift
    /*
    For now, we are taking the Allen Covariance as an indicator of
    noise on the EKF output from the IMU.
    */
    // accel_bias_.x = accel_bias_noise_();
    // accel_bias_.y = accel_bias_noise_();
    // accel_bias_.z = accel_bias_noise_();

    // gyro_bias_.x = gyro_bias_noise_();
    // gyro_bias_.y = gyro_bias_noise_();
    // gyro_bias_.z = gyro_bias_noise_();

    accel.x = std::round(((curr_velocity.x - last_velocity_.x) / dt + diff_accel_(rng_generator_) +
                          accel_bias_noise_()) /
                         resolution_) *
              resolution_;
    accel.y = std::round(((curr_velocity.y - last_velocity_.y) / dt + diff_accel_(rng_generator_) +
                          accel_bias_noise_()) /
                         resolution_) *
              resolution_;
    accel.z = std::round(((curr_velocity.z - last_velocity_.z) / dt + diff_accel_(rng_generator_) +
                          accel_bias_noise_()) /
                         resolution_) *
              resolution_;

    noisy_msg.linear_acceleration = accel;

    // add noise to angular velocity measurement
    noisy_msg.angular_velocity.x = std::round((msg.twist.twist.angular.x +
                                               diff_gyro_(rng_generator_) + gyro_bias_noise_()) /
                                              resolution_) *
                                   resolution_;
    noisy_msg.angular_velocity.y = std::round((msg.twist.twist.angular.y +
                                               diff_gyro_(rng_generator_) + gyro_bias_noise_()) /
                                              resolution_) *
                                   resolution_;
    noisy_msg.angular_velocity.z = std::round((msg.twist.twist.angular.z +
                                               diff_gyro_(rng_generator_) + gyro_bias_noise_()) /
                                              resolution_) *
                                   resolution_;

    // add noise to orientation measurement
    // noisy_msg.orientation.x = std::round((msg.pose.pose.orientation.x + diff_gyro_(rng_generator_)) / resolution_) * resolution_;
    // noisy_msg.orientation.y = std::round((msg.pose.pose.orientation.y + diff_gyro_(rng_generator_)) / resolution_) * resolution_;
    // noisy_msg.orientation.z = std::round((msg.pose.pose.orientation.z + diff_gyro_(rng_generator_)) / resolution_) * resolution_;
    // noisy_msg.orientation.w = std::round((msg.pose.pose.orientation.w + diff_gyro_(rng_generator_)) / resolution_) * resolution_;

    // add the covariances
    // MAY NEED THIS LATER SO NOT DELETING IT. add covariance for state estimation pipeline. Since there is an EKF on the sensor itself, this should be provided for us
    noisy_msg.orientation_covariance[0] = -1;
    noisy_msg.orientation_covariance[4] = -1;
    noisy_msg.orientation_covariance[8] = -1;

    noisy_msg.angular_velocity_covariance[0] = gyro_cov_;
    noisy_msg.angular_velocity_covariance[4] = gyro_cov_;
    noisy_msg.angular_velocity_covariance[8] = gyro_cov_;

    noisy_msg.linear_acceleration_covariance[0] = accel_cov_;
    noisy_msg.linear_acceleration_covariance[4] = accel_cov_;
    noisy_msg.linear_acceleration_covariance[8] = accel_cov_;

    // Update for next iteration
    last_velocity_ = curr_velocity;
    last_time_ = current_time;

    // publish the message
    measurement_publisher->publish(noisy_msg);
  }

  // function that adds bias noise to a measurement:
  double accel_bias_noise_()
  {
    std::normal_distribution<double> bias_dist(0.0, accel_bias_stability_ * 9.81 / 1000);
    return bias_dist(rng_generator_);
  }

  double gyro_bias_noise_()
  {
    std::normal_distribution<double> bias_dist(0.0, gyro_bias_stability_ * (M_PI / 180) / 3600);
    return bias_dist(rng_generator_);
  }
  // declare ros pubsub
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr measurement_publisher;

  // parameter variables(imported from sensors.launch.yaml)
  double accel_noise_density_;
  int sample_rate_;
  double gyro_noise_density_;
  double accel_bias_stability_;
  double gyro_bias_stability_;
  double bias_drift_rate_;
  double total_drift_;
  double bias_;
  double resolution_;
  double resolution_mms_;
  bool measure_ang_twist_;

  // Extra variables needed for the sim
  double accel_cov_;
  double gyro_cov_;
  double accel_stddev_;
  double gyro_stddev_;
  geometry_msgs::msg::Vector3 last_velocity_; // to derive acceleration
  // geometry_msgs::msg::Vector3 accel_bias_;
  // geometry_msgs::msg::Vector3 gyro_bias_;
  rclcpp::Time last_time_;
  bool first_message_;
  std::string input_topic_name_;
  std::string output_topic_name_;
  std::default_random_engine rng_generator_;
  std::normal_distribution<double> diff_accel_;
  std::normal_distribution<double> diff_gyro_;
};

// define the main function which actually spins the sensor up
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VectornavIMU>());
  rclcpp::shutdown();
  return 0;
}
