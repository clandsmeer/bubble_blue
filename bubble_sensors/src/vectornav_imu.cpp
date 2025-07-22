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

class VectornavIMU : public rclcpp::Node
{
public:
    VectornavIMU() : Node("vectornav_imu")
    {
        // create/read the parameters for the sensor in the constructor:
        //  standard deviation
        this->declare_parameter<double>("noise_stddev", 0.05);
        this->get_parameter("noise_stddev", noise_stddev_);

        // defining which measurements will be noisy
        this->declare_parameter<double>("bias", 0.05);
        this->get_parameter("bias", bias_);

        this->declare_parameter<double>("bias_drift", 0.05);
        this->get_parameter("bias_drift", bias_drift_rate_);

        this->declare_parameter<double>("resolution", 0.1);
        this->get_parameter("resolution", resolution_mms_);

        this->declare_parameter<bool>("angular_twist", false);
        this->get_parameter("angular_twist", measure_ang_twist_);

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
        cov_ = noise_stddev_ * noise_stddev_ + bias_ * bias_ + bias_drift_rate_ * bias_drift_rate_;
        resolution_ = resolution_mms_ / 1000;
        first_message_ = true;

        RCLCPP_INFO(this->get_logger(), "Noisy odometry relay started.");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_name_.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry &msg)
    {
        // create the variable that will store the new noisy message
        sensor_msgs::msg::Imu noisy_msg;

        // create a gaussian distribution. Zero-mean w/ specified stddev
        std::normal_distribution diff(0.0, noise_stddev_);

        // first add the header
        noisy_msg.header = msg.header;

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

        accel.x = std::round(((curr_velocity.x - last_velocity_.x) / dt + diff(rng_generator_)) / resolution_) * resolution_;
        accel.y = std::round(((curr_velocity.y - last_velocity_.y) / dt + diff(rng_generator_)) / resolution_) * resolution_;
        accel.z = std::round(((curr_velocity.z - last_velocity_.z) / dt + diff(rng_generator_)) / resolution_) * resolution_;

        noisy_msg.linear_acceleration = accel;

        // add noise to angular velocity measurement
        noisy_msg.angular_velocity.x = std::round((msg.twist.twist.angular.x + diff(rng_generator_)) / resolution_) * resolution_;
        noisy_msg.angular_velocity.y = std::round((msg.twist.twist.angular.y + diff(rng_generator_)) / resolution_) * resolution_;
        noisy_msg.angular_velocity.z = std::round((msg.twist.twist.angular.z + diff(rng_generator_)) / resolution_) * resolution_;

        // add noise to orientation measurement
        noisy_msg.orientation.x = std::round((msg.pose.pose.orientation.x + diff(rng_generator_)) / resolution_) * resolution_;
        noisy_msg.orientation.y = std::round((msg.pose.pose.orientation.y + diff(rng_generator_)) / resolution_) * resolution_;
        noisy_msg.orientation.z = std::round((msg.pose.pose.orientation.z + diff(rng_generator_)) / resolution_) * resolution_;
        noisy_msg.orientation.w = std::round((msg.pose.pose.orientation.w + diff(rng_generator_)) / resolution_) * resolution_;

        // add the covariances
        // MAY NEED THIS LATER SO NOT DELETING IT. add covariance for state estimation pipeline. Since there is an EKF on the sensor itself, this should be provided for us
        noisy_msg.orientation_covariance[0] = cov_;
        noisy_msg.orientation_covariance[4] = cov_;
        noisy_msg.orientation_covariance[8] = cov_;

        noisy_msg.angular_velocity_covariance[0] = cov_;
        noisy_msg.angular_velocity_covariance[4] = cov_;
        noisy_msg.angular_velocity_covariance[8] = cov_;

        noisy_msg.linear_acceleration_covariance[0] = cov_;
        noisy_msg.linear_acceleration_covariance[4] = cov_;
        noisy_msg.linear_acceleration_covariance[8] = cov_;

        // Update for next iteration
        last_velocity_ = curr_velocity;
        last_time_ = current_time;

        // publish the message
        measurement_publisher->publish(noisy_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr measurement_publisher;
    double noise_stddev_;
    double bias_drift_rate_;
    double bias_;
    double cov_;
    double resolution_;
    double resolution_mms_;
    bool measure_ang_twist_;
    geometry_msgs::msg::Vector3 last_velocity_;
    rclcpp::Time last_time_;
    bool first_message_;
    std::string input_topic_name_;
    std::string output_topic_name_;
    std::default_random_engine rng_generator_;
};

// define the main function which actually spins the sensor up
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectornavIMU>());
    rclcpp::shutdown();
    return 0;
}
