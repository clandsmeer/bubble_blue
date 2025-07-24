#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>

class GenericSensor : public rclcpp::Node
{
public:
    GenericSensor() : Node("generic_sensor")
    {
        // create the parameters for the sensor:
        //  standard deviation
        this->declare_parameter<double>("noise_stddev", 0.05);
        this->get_parameter("noise_stddev", noise_stddev_);

        // defining which measurements will be noisy
        this->declare_parameter<bool>("position_meas", true);
        this->get_parameter("position_meas", measure_position);

        this->declare_parameter<bool>("angular_meas", true);
        this->get_parameter("angular_meas", measure_ang_orient);

        this->declare_parameter<bool>("pos_twist", true);
        this->get_parameter("pos_twist", measure_pos_twist);

        this->declare_parameter<bool>("angular_twist", true);
        this->get_parameter("angular_twist", measure_ang_twist);

        // Define the incoming and outgoing topics
        this->declare_parameter("input_topic", "/model/bluerov2/odometry");
        this->get_parameter("input_topic", input_topic_name);

        this->declare_parameter("output_topic", "/sensors/generalized_sensor_test");
        this->get_parameter("output_topic", output_topic_name);

        odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic_name,
            10,
            std::bind(&GenericSensor::odom_callback, this, std::placeholders::_1));

        measurement_publisher = this->create_publisher<nav_msgs::msg::Odometry>(
            output_topic_name,
            10);

        RCLCPP_INFO(this->get_logger(), "Noisy odometry relay started.");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", input_topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", output_topic_name.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry &msg)
    {
        // create the variable that will store the new noisy message
        auto noisy_msg = msg;

        // create a gaussian distribution
        std::normal_distribution diff(0.0, noise_stddev_);

        // adding noise to the measurement depending on which
        if (measure_position)
        {
            // If sensor measures position, add noise to position
            noisy_msg.pose.pose.position.x += diff(rng_generator);
            noisy_msg.pose.pose.position.y += diff(rng_generator);
            noisy_msg.pose.pose.position.z += diff(rng_generator);
        }

        if (measure_ang_orient)
        {
            // If sensor measures position, add noise to position
            noisy_msg.pose.pose.orientation.x += diff(rng_generator);
            noisy_msg.pose.pose.orientation.y += diff(rng_generator);
            noisy_msg.pose.pose.orientation.z += diff(rng_generator);
        }

        if (measure_pos_twist)
        {
            noisy_msg.twist.twist.linear.x += diff(rng_generator);
            noisy_msg.twist.twist.linear.y += diff(rng_generator);
            noisy_msg.twist.twist.linear.z += diff(rng_generator);
        }

        if (measure_ang_twist)
        {
            noisy_msg.twist.twist.angular.x += diff(rng_generator);
            noisy_msg.twist.twist.angular.y += diff(rng_generator);
            noisy_msg.twist.twist.angular.z += diff(rng_generator);
        }

        measurement_publisher->publish(noisy_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr measurement_publisher;
    double noise_stddev_;
    bool measure_position;
    bool measure_ang_orient;
    bool measure_pos_twist;
    bool measure_ang_twist;
    std::string input_topic_name;
    std::string output_topic_name;
    std::default_random_engine rng_generator;
};

// define the main function which actually spins the sensor up
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenericSensor>());
    rclcpp::shutdown();
    return 0;
}
