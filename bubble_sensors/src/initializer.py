#!/usr/bin/env python3
"""
File: initializer v1.0
Author: Henry Adam
Date: Aug 19, 2025

The purpose of this file is to complete the automated initialization of each of the sensos
on the BlueRov2. It is called via the /initialize service of type std_srvs/srv/Trigger.
"""

import rclpy
from dvl_msgs.msg import ConfigCommand
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool

from bubble_sensors.srv import ConfigureVN100


class Initializer(Node):
    def __init__(self):
        super().__init__('initializer')

        #import all the ros parameters
        self.declare_parameter("port1_data_register", 16)
        self.declare_parameter("port1_frequency", 50)

        self.declare_parameter("port2_data_register", 254)
        self.declare_parameter("port2_frequency", 50)

        self.declare_parameter("gp_origin_lat", 47.3769)
        self.declare_parameter("gp_origin_long", 8.5417)
        self.declare_parameter("gp_origin_alt", 0.0)

        self.declare_parameter("init_pose_x", 0.0)
        self.declare_parameter("init_pose_y", 0.0)
        self.declare_parameter("init_pose_z", 0.0)

        self.declare_parameter("rot_matrix_imu_to_body", [1.0,0.0,0.0,0.0,0.0,-1.0,0.0,1.0,0.0])

        self.declare_parameter("mag_ref_x", 0.22) #Zurich Magnetic Field
        self.declare_parameter("mag_ref_y", 0.03)
        self.declare_parameter("mag_ref_z", 0.89)

        self.declare_parameter("gravity", 9.80600) #Zurich Gravitational Acceleration
        #Get all the parameters
        self.port1_data_register = self.get_parameter("port1_data_register").get_parameter_value().integer_value
        self.port1_frequency = self.get_parameter("port1_frequency").get_parameter_value().integer_value

        self.port2_data_register = self.get_parameter("port2_data_register").get_parameter_value().integer_value
        self.port2_frequency = self.get_parameter("port2_frequency").get_parameter_value().integer_value

        self.gp_origin_lat = self.get_parameter("gp_origin_lat").get_parameter_value().double_value
        self.gp_origin_long = self.get_parameter("gp_origin_long").get_parameter_value().double_value
        self.gp_origin_alt = self.get_parameter("gp_origin_alt").get_parameter_value().double_value

        self.init_pose_x = self.get_parameter("init_pose_x").get_parameter_value().double_value
        self.init_pose_y = self.get_parameter("init_pose_y").get_parameter_value().double_value
        self.init_pose_z = self.get_parameter("init_pose_z").get_parameter_value().double_value

        self.imu_orientation = self.get_parameter("rot_matrix_imu_to_body").get_parameter_value().double_array_value

        self.mag_ref_x = self.get_parameter("mag_ref_x").get_parameter_value().double_value
        self.mag_ref_y = self.get_parameter("mag_ref_y").get_parameter_value().double_value
        self.mag_ref_z = self.get_parameter("mag_ref_z").get_parameter_value().double_value

        self.gravity = self.get_parameter("gravity").get_parameter_value().double_value

        # Create the client for the IMU Configuration service
        self.config_cli = self.create_client(ConfigureVN100, '/configure_vn100')
        while not self.config_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for /configure_vn100 service...')

        # Create the client for the Bias Estimation  Service
        self.bias_cli = self.create_client(Trigger, '/estimate_bias')
        while not self.bias_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for /estimate_bias service...')

        # Create the client for the Port Publishing Service
        self.port_publishing_cli = self.create_client(Trigger, '/set_reading_status')
        while not self.port_publishing_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for /set_reading_status service...')

        # Publishers for global and local positions
        self.global_pub = self.create_publisher(GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)
        self.local_pub = self.create_publisher(PoseWithCovarianceStamped, '/mavros/local_position/pose_cov', 10)
        self.dvl_publisher = self.create_publisher(ConfigCommand, '/dvl/config/command', 10)

        # Subscribe to the Imu_body topic to account for bias(simple averaging)
        self.Imu_sub = self.create_subscription(Imu, "/vectornav/Imu_body", self.imu_sub_callback, 10)

        # create variables for rolling average

        #create the triggerred service that initailizes the system.
        self.init_service = self.create_service(Trigger, "initialize", self.init_service_callback)
        self.get_logger().info("Initializer Started and /configure_vn100 service grabbed. Ready for initialization")

    def init_service_callback(self, request, response):
        #the purpose of this service is to initialize the imu, dvl, and ardusub ek3 output

        #first, send the imu the command to output the correct data from the correct ports
        req_port1_dataType = ConfigureVN100.Request()
        req_port1_dataType.port = "port1"
        req_port1_dataType.msg = f"VNWRG,06,{self.port1_data_register},1"#True body-fixed accelerations
        response_port1_dataType = self.config_cli.call_async(req_port1_dataType)
        self.get_logger().info(f'Sent configuration command for port 1 output data type. Response: {response_port1_dataType.result()}')

        req_port1_dataRate = ConfigureVN100.Request()
        req_port1_dataRate.port = "port1"
        req_port1_dataRate.msg = f"VNWRG,07,{self.port1_frequency},1" #50Hz
        response_port1_dataRate = self.config_cli.call_async(req_port1_dataRate)
        self.get_logger().info(f'Sent configuration command for port 1 output data rate. Response: {response_port1_dataRate.result()}')

        req_port2_dataType = ConfigureVN100.Request()
        req_port2_dataType.port = "port1"
        req_port2_dataType.msg = f"VNWRG,06,{self.port2_data_register},2" #kalman-filterred output
        response_port2_dataType = self.config_cli.call_async(req_port2_dataType)
        self.get_logger().info(f'Sent configuration command for port 2 output data type. Response: {response_port2_dataType.result()}')

        req_port2_dataRate = ConfigureVN100.Request()
        req_port2_dataRate.port = "port1"
        req_port2_dataRate.msg = f"VNWRG,07,{self.port2_frequency},2"
        response_port2_dataRate = self.config_cli.call_async(req_port2_dataRate)
        self.get_logger().info(f'Sent configuration command for port 2 output data rate. Response: {response_port2_dataRate.result()}')

        #NOTE: Orientation Specification is saved into the IMU itself. Please set this
        #manually using the register 26 if the mounting of the IMU changes.

        # set the correct gravity vector, they use a slightly smaller one
        req_gravity = ConfigureVN100.Request()
        req_gravity.port = "port1"
        req_gravity.msg = f"VNWRG,21,{self.mag_ref_x},{self.mag_ref_y},{self.mag_ref_z},0,0,{self.gravity}"
        response_req_gravity = self.config_cli.call_async(req_gravity)
        self.get_logger().info(f'Sent configuration command for IMU gravity. Response: {response_req_gravity.result()}')

        # trigger the bias estimation service, use call() to block the reading process
        trigger_bias_est = Trigger.Request()
        response_bias_est = self.bias_cli.call(trigger_bias_est)
        if response_bias_est is not None:
            self.get_logger().info(f'Sent command to begin bias estimation process. Response: {response_bias_est.result()}')
        else:
            self.get_logger().error('Failed to complete bias estimation service')

        # After the bias estimation service has completed, start the publishing service for the IMU
        publishing_request = SetBool.Request()
        publishing_request.data = True
        response_publishing_request = self.port_publishing_cli.call_async(publishing_request)
        self.get_logger().info(f'Sent command to begin bias estimation process. Response: {response_publishing_request.result()}')

        # Next, send the global/local reference positions
        self.publish_reference_positions()

        #Finally, enable accoustic for the dvl
        self.send_dvl_enable()

        # TODO: More sophisticated handling of service response
        response.success = True
        response.message = "PLACEHOLDER MESSAGE FOR SERVICE STATUS FOR INITIAL TESITNG. MUST UPDATE."
        return response

    def send_dvl_enable(self):
        msg = ConfigCommand()
        msg.command = 'set_config'
        msg.parameter_name = 'acoustic_enabled'
        msg.parameter_value = 'true'
        self.dvl_publisher.publish(msg)
        self.get_logger().info('Sent configuration command to enable dvl.')
        #TODO: Add a check of the status topic to check if this has been carried out.

    def publish_reference_positions(self):
        # Reference global position (example values)
        global_msg = GeoPointStamped()
        global_msg.header.stamp = self.get_clock().now().to_msg()
        global_msg.position.latitude = self.gp_origin_lat
        global_msg.position.longitude = self.gp_origin_long
        global_msg.position.altitude = self.gp_origin_alt
        self.global_pub.publish(global_msg)
        self.get_logger().info('Published reference global position.')

        # Set reference local position to zero
        local_msg = PoseWithCovarianceStamped()
        local_msg.header.frame_id = "map"
        local_msg.header.stamp = self.get_clock().now().to_msg()
        local_msg.pose.pose.position.x = 0.0
        local_msg.pose.pose.position.y = 0.0
        local_msg.pose.pose.position.z = 0.0
        local_msg.pose.pose.orientation.w = 1.0
        self.local_pub.publish(local_msg)
        self.get_logger().info('Published reference local position.')

        #TODO: Add automatic checking that this worked and output it as a boolean

        return

    # Update a rolling average
    def imu_sub_callback(self, msg):
        return

def main(args=None):
    rclpy.init(args=args)
    node = Initializer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
