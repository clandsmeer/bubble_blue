#include <bubble_sensors/mavros_sensor_plugin.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*
This script is an example for a mavros sensor plugin. It demonstrates both writing
ros topic data to pre-defined mavros nodes AND writing mavlink messages directly 
to the mavlink master. The pose(position/attitude) will be written to the mavros/vision_pose/pose 
topic, while the twist will be written to the mavlink VISUAL_SPEED_ESTIMATE variable, which 
eventually should show up in the /mavros/vision_pose/pose_cov class
*/
namespace bubble_sensors { 

    MavrosSensorPlugin::MavrosSensorPlugin(mavros::plugin::UASPtr uas_) : mavros::plugin::Plugin(uas_, "mavros_sensor"){
        // use mavros' node_declare_and_watch_parameter so you can change the parameter during runtime
        /*node_declare_and_watch_parameter("sensor_topic", "/sensors/generalized_sensor_test", [&](const rclcpp::Parameter &p){
            sensor_topic_ = p.as_string();
        });

        node_declare_and_watch_parameter(
            "mavros_topic", "/mavros/vision_speed/speed_twist", [&](const rclcpp::Parameter & p) {
                mavros_topic_ = p.as_string();
            });
        
        node_declare_and_watch_parameter(
            "publish_mavlink", true, [&](const rclcpp::Parameter & p) {
                publish_mavlink_ = p.as_bool();
            });*/

        auto node_handle = node; 
        // Declare parameters with default values
        node_handle->declare_parameter<std::string>("mavros_sensor.sensor_topic", "/sensors/generalized_sensor_test");
        node_handle->declare_parameter<std::string>("mavros_sensor.mavros_topic", "/mavros/custom_vision_speed/speed_twist");
        node_handle->declare_parameter<bool>("mavros_sensor.publish_mavlink", true);

        // Get the parameters
        sensor_topic_ = node_handle->get_parameter("mavros_sensor.sensor_topic").as_string();
        mavros_topic_ = node_handle->get_parameter("mavros_sensor.mavros_topic").as_string();
        publish_mavlink_ = node_handle->get_parameter("mavros_sensor.publish_mavlink").as_bool();

        // Subscribe to the generic sensor ROS topic
        generic_sensor_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            sensor_topic_, 
            10,
            std::bind(&MavrosSensorPlugin::generic_sensor_callback, this, std::placeholders::_1)
        ); 
        
        // Publish to the ROS topics that are available
        local_velocity_pub_ = node->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            mavros_topic_, 10
        );

        // Log initialization
        RCLCPP_INFO(node->get_logger(), "MavrosSensorPlugin initialized");
        RCLCPP_INFO(node->get_logger(), "Subscribing to: %s", sensor_topic_.c_str());
        RCLCPP_INFO(node->get_logger(), "Publishing to: %s", mavros_topic_.c_str());
    }

    MavrosSensorPlugin::Subscriptions MavrosSensorPlugin::get_subscriptions() {
        return {}; //not subscribing to any MAVLink nodes since this is just a publisher
    }
    
    void MavrosSensorPlugin::generic_sensor_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        RCLCPP_DEBUG(node->get_logger(), "Received sensor data, publishing to %s", mavros_topic_.c_str());
        
        // Publish the twist message
        auto twist_msg = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();

        // Fill in the data
        twist_msg->header = msg->header;
        twist_msg->header.frame_id = "odom"; 
        twist_msg->twist.twist = msg->twist.twist; 

        local_velocity_pub_->publish(*twist_msg); 

        if (publish_mavlink_){
            mavlink::common::msg::VICON_POSITION_ESTIMATE vse{};
            uint64_t time_usec = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000ULL;
            vse.usec = time_usec; 

            //gather velocity and add it to the message
            vse.x = msg->twist.twist.linear.x;
            vse.y = msg->twist.twist.linear.y; 
            vse.z = msg->twist.twist.linear.z;
            
            //actually send message now
            uas->send_message(vse); 
        }
    }
}

#include <mavros/mavros_plugin_register_macro.hpp>
MAVROS_PLUGIN_REGISTER(bubble_sensors::MavrosSensorPlugin)