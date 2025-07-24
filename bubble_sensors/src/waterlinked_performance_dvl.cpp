/*
waterlinked_performance_dvl.cpp V1.0
Author: Henry Adam
Purpose:
This file is meant to act as the node representing the waterlinked performance DVL
for the Blue simulation used by Bubble Robotics. It follows the referred specs of the
DVL at this link: https://waterlinked.com/datasheets/dvl-a50
*/

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <random>

class PerformanceDVL : public rclcpp::Node
{
public:
  PerformanceDVL()
  : Node("perforamnce_dvl")
  {
        // create/read the parameters for the sensor in the constructor:
        //  standard deviation
    this->declare_parameter<double>("noise_stddev", 0.05);
    this->get_parameter("noise_stddev", noise_stddev_);

        // defining which measurements will be noisy
    this->declare_parameter<double>("bias", 0.05);
    this->get_parameter("bias", bias_);

    this->declare_parameter<double>("long_term_inaccuracy_percentage", 0.1);
    this->get_parameter("long_term_inaccuracy_percentage", long_term_inaccuracies_);

    this->declare_parameter<double>("resolution_mms", 0.1);
    this->get_parameter("resolution_mms", resolution_mms_);

    this->declare_parameter<bool>("angular_twist", false);
    this->get_parameter("angular_twist", measure_ang_twist_);

        // Define the incoming and outgoing topics
    this->declare_parameter("input_topic", "/model/bluerov2/odometry");
    this->get_parameter("input_topic", input_topic_name_);

    this->declare_parameter("output_topic", "/dvl/velocity");
    this->get_parameter("output_topic", output_topic_name_);

    odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic_name_,
            10,
            std::bind(&PerformanceDVL::odom_callback, this, std::placeholders::_1));

    measurement_publisher = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            output_topic_name_,
            10);

        // Declare the covariance
    cov_ = noise_stddev_ * noise_stddev_ + bias_ * bias_ + (long_term_inaccuracies_ / 100) *
      (long_term_inaccuracies_ / 100);
    resolution_ = resolution_mms_ / 1000;
        // seed the random generator:
    std::srand(std::time(0));     // seed w/ start time of program

    RCLCPP_INFO(this->get_logger(), "Noisy odometry relay started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_name_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
        // create the variable that will store the new noisy message
    geometry_msgs::msg::TwistWithCovarianceStamped noisy_msg;

        // create a gaussian distribution
    std::normal_distribution diff(0.0, noise_stddev_);

        // first add the header
    noisy_msg.header = msg.header;

        // compensate for long-term inaccuracies in the twist measurement.
    inaccurate_linear_twist_.x = msg.twist.twist.linear.x *
      (long_term_inaccuracies_ * (double)std::rand() / RAND_MAX);
    inaccurate_linear_twist_.y = msg.twist.twist.linear.y *
      (long_term_inaccuracies_ * (double)std::rand() / RAND_MAX);
    inaccurate_linear_twist_.z = msg.twist.twist.linear.z *
      (long_term_inaccuracies_ * (double)std::rand() / RAND_MAX);

        // now add some noise
    noisy_msg.twist.twist.linear.x += std::round((inaccurate_linear_twist_.x +
        diff(rng_generator_)) / resolution_) * resolution_;
    noisy_msg.twist.twist.linear.y += std::round((inaccurate_linear_twist_.y +
        diff(rng_generator_)) / resolution_) * resolution_;
    noisy_msg.twist.twist.linear.z += std::round((inaccurate_linear_twist_.z +
        diff(rng_generator_)) / resolution_) * resolution_;

        // add covariance for state estimation pipeline. Since there is an EKF on the sensor itself, this should be provided for us
    noisy_msg.twist.covariance[0] = cov_;
    noisy_msg.twist.covariance[7] = cov_;
    noisy_msg.twist.covariance[14] = cov_;

        // dont think dvl measures angular rates but just in case
    if (measure_ang_twist_) {
      noisy_msg.twist.twist.angular.x += std::round(msg.twist.twist.angular.x +
        diff(rng_generator_) / resolution_) * resolution_;
      noisy_msg.twist.twist.angular.y += std::round(msg.twist.twist.angular.y +
        diff(rng_generator_) / resolution_) * resolution_;
      noisy_msg.twist.twist.angular.z += std::round(msg.twist.twist.angular.z +
        diff(rng_generator_) / resolution_) * resolution_;

            // add covariance for state estimation pipeline
      noisy_msg.twist.covariance[21] = cov_;
      noisy_msg.twist.covariance[28] = cov_;
      noisy_msg.twist.covariance[35] = cov_;
    } else {
            // make cov -1 to identify that it should not be used
      noisy_msg.twist.covariance[21] = -1;
      noisy_msg.twist.covariance[28] = -1;
      noisy_msg.twist.covariance[35] = -1;
    }

        // publish the message
    measurement_publisher->publish(noisy_msg);
  }

    // Declaring the ROS nodes
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr measurement_publisher;

    // Parameters
  double noise_stddev_;
  double long_term_inaccuracies_;
  double bias_;
  double cov_;
  double resolution_;
  double resolution_mms_;
  bool measure_ang_twist_;
  std::string input_topic_name_;
  std::string output_topic_name_;

    // ancillary variables
  std::default_random_engine rng_generator_;
  geometry_msgs::msg::Vector3 inaccurate_linear_twist_;
};

// define the main function which actually spins the sensor up
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerformanceDVL>());
  rclcpp::shutdown();
  return 0;
}
