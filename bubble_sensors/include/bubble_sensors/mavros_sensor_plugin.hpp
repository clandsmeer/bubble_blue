#pragma once

#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bubble_sensors
{
    class MavrosSensorPlugin : public mavros::plugin::Plugin
    {
    public:
        explicit MavrosSensorPlugin(mavros::plugin::UASPtr uas_);

        // getting subscriptions not necessary, but need to declare since it is a required member
        MavrosSensorPlugin::Subscriptions get_subscriptions() override;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr generic_sensor_sub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr local_velocity_pub_;

        std::string sensor_topic_;
        std::string mavros_topic_;
        bool publish_mavlink_;

        void generic_sensor_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    };
}
