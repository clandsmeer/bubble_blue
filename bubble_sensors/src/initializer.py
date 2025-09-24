#!/usr/bin/env python3
"""
File: initializer v3.0.

Author: Henry Adam
Date: Aug 19, 2025

The purpose of this file is to complete the automated initialization of each
of the sensos on the BlueRov2. It is called via the /initialize service of type
std_srvs/srv/Trigger.
"""

import time

from bubble_sensors.srv import ConfigureVN100
from dvl_msgs.msg import ConfigCommand
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


class Initializer(Node):
    def __init__(self):
        super().__init__('initializer')

        # import all the ros parameters
        self.declare_parameter('port1_data_register', 16)
        self.declare_parameter('port1_frequency', 50)

        self.declare_parameter('port2_data_register', 254)
        self.declare_parameter('port2_frequency', 50)

        self.declare_parameter('gp_origin_lat', 47.3769)
        self.declare_parameter('gp_origin_long', 8.5417)
        self.declare_parameter('gp_origin_alt', 0.0)

        self.declare_parameter('init_pose_x', 0.0)
        self.declare_parameter('init_pose_y', 0.0)
        self.declare_parameter('init_pose_z', 0.0)

        self.declare_parameter('mag_ref_x', 0.22)  # Zurich Magnetic Field
        self.declare_parameter('mag_ref_y', 0.03)
        self.declare_parameter('mag_ref_z', 0.89)

        self.declare_parameter('gravity', 9.80600)  # Zurich Gravitational Acceleration

        self.declare_parameter('correct_gravity', True)
        # Whether or not to send the bias estimate message
        self.declare_parameter('correct_bias', False)
        self.declare_parameter('use_imu', True)

        # Get all the parameters
        self.port1_data_register = self.get_parameter(
            'port1_data_register').get_parameter_value().integer_value
        self.port1_frequency = self.get_parameter(
            'port1_frequency').get_parameter_value().integer_value

        self.port2_data_register = self.get_parameter(
            'port2_data_register').get_parameter_value().integer_value
        self.port2_frequency = self.get_parameter(
            'port2_frequency').get_parameter_value().integer_value

        self.gp_origin_lat = self.get_parameter(
            'gp_origin_lat').get_parameter_value().double_value
        self.gp_origin_long = self.get_parameter(
            'gp_origin_long').get_parameter_value().double_value
        self.gp_origin_alt = self.get_parameter(
            'gp_origin_alt').get_parameter_value().double_value

        self.init_pose_x = self.get_parameter(
            'init_pose_x').get_parameter_value().double_value
        self.init_pose_y = self.get_parameter(
            'init_pose_y').get_parameter_value().double_value
        self.init_pose_z = self.get_parameter(
            'init_pose_z').get_parameter_value().double_value

        self.mag_ref_x = self.get_parameter(
            'mag_ref_x').get_parameter_value().double_value
        self.mag_ref_y = self.get_parameter(
            'mag_ref_y').get_parameter_value().double_value
        self.mag_ref_z = self.get_parameter(
            'mag_ref_z').get_parameter_value().double_value

        self.gravity = self.get_parameter(
            'gravity').get_parameter_value().double_value

        self.correct_gravity = self.get_parameter(
            'correct_gravity').get_parameter_value().bool_value

        self.correct_bias = self.get_parameter(
            'correct_bias').get_parameter_value().bool_value

        self.use_imu = self.get_parameter(
            'use_imu').get_parameter_value().bool_value

        # Create the client for the IMU Configuration service
        if self.use_imu:
            client_cb_group = MutuallyExclusiveCallbackGroup()
            self.config_cli = self.create_client(
                ConfigureVN100,
                '/configure_vn100', callback_group=client_cb_group)
            while not self.config_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Waiting for /configure_vn100 service...')

            # Create the client for the Bias Estimation  Service
            self.bias_cli = self.create_client(Trigger, '/estimate_bias')
            while not self.bias_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Waiting for /estimate_bias service...')

            # Create the client for the Port Publishing Service
            self.port_publishing_cli = self.create_client(
                SetBool,
                '/set_reading_status')
            while not self.port_publishing_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Waiting for /set_reading_status service...')

        # Publishers for global and local positions
        self.global_pub = self.create_publisher(
            GeoPointStamped,
            '/mavros/global_position/set_gp_origin', 10)

        self.local_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/set_pose', 10)

        self.dvl_publisher = self.create_publisher(
            ConfigCommand,
            '/dvl/config/command', 10)

        init_callbackGroup = MutuallyExclusiveCallbackGroup()
        # create the triggered service that initializes the system.
        self.init_service = self.create_service(
            Trigger,
            'initialize',
            self.init_service_callback,
            callback_group=init_callbackGroup)

        self.get_logger().info(
            'Initializer Started and /configure_vn100 service grabbed. '
            'Ready for initialization')

        # Add the variables to be used by the future callbacks to indicate completion
        self.port1_dataType_set = False
        self.port1_dataRate_set = False
        self.port2_dataType_set = False
        self.port2_dataRate_set = False
        self.mag_gravity_set = False
        self.bias_calculated = False
        self.ports_publishing = False

    def init_service_callback(self, request, response):
        # the purpose of this service is to initialize
        # the imu, dvl, and ardusub ek3 output

        if self.use_imu:
            self.configure_imu()

        # Next, send the global/local reference positions
        self.publish_reference_positions()

        # Finally, enable acoustic for the dvl
        self.send_dvl_enable()

        # TODO: More sophisticated handling of service response
        response.success = True
        response.message = 'PLACEHOLDER MESSAGE FOR SERVICE STATUS.'
        return response

    # Configure the IMU by sending consecutive messages
    def configure_imu(self):
        # pre-set these to true and switch to false if they fail
        Port1Data_Success = True
        Port1DataRate_Success = True
        Port2Data_Success = True
        Port2DataRate_Success = True
        Gravity_Success = True
        BiasCorrection_Success = True
        PortPublishing_Success = True

        # Port 1 data type
        if not self.call_service_and_wait(
            self.config_cli,
            ConfigureVN100.Request(
                port='port1',
                msg=f'VNWRG,06,{self.port1_data_register},1'
            ),
            'Port 1 Data Type'
        ):
            Port1Data_Success = False

        # Port 1 data rate
        if not self.call_service_and_wait(
            self.config_cli,
            ConfigureVN100.Request(
                port='port1',
                msg=f'VNWRG,07,{self.port1_frequency},1'
            ),
            'Port 1 Data Rate'
        ):
            Port1DataRate_Success = False

        # Port 2 data type
        if not self.call_service_and_wait(
            self.config_cli,
            ConfigureVN100.Request(
                port='port1',
                msg=f'VNWRG,06,{self.port2_data_register},2'
            ),
            'Port 2 Data Type'
        ):
            Port2Data_Success = False

        # Port 2 data rate
        if not self.call_service_and_wait(
            self.config_cli,
            ConfigureVN100.Request(
                port='port1',
                msg=f'VNWRG,07,{self.port2_frequency},2'
            ),
            'Port 2 Data Rate'
        ):
            Port2DataRate_Success = False

        # Set gravity and magnetic reference
        if self.correct_gravity:
            if not self.call_service_and_wait(
                self.config_cli,
                ConfigureVN100.Request(
                    port='port1',
                    msg=(f'VNWRG,21,{self.mag_ref_x},{self.mag_ref_y},{self.mag_ref_z},'
                         f'0,0,{self.gravity}')
                ),
                'Gravity and Magnetic Reference'
            ):
                Gravity_Success = False

        # Bias estimation
        if self.correct_bias:
            if not self.call_service_and_wait(
                self.bias_cli,
                Trigger.Request(),
                'Bias Estimation',
                timeout=60.0
            ):
                BiasCorrection_Success = False

        # Start port publishing
        if not self.call_service_and_wait(
            self.port_publishing_cli,
            SetBool.Request(data=True),
            'Port Publishing'
        ):
            PortPublishing_Success = False

        return (Port1Data_Success and
                Port1DataRate_Success and
                Port2Data_Success and
                Port2DataRate_Success and
                Gravity_Success and
                BiasCorrection_Success and
                PortPublishing_Success)

    def send_dvl_enable(self):
        msg = ConfigCommand()
        msg.command = 'set_config'
        msg.parameter_name = 'acoustic_enabled'
        msg.parameter_value = 'true'
        self.dvl_publisher.publish(msg)
        self.get_logger().info('Sent configuration command to enable dvl.')
        # TODO: Add a check of the status topic to check if this has been carried out.

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
        local_msg.header.frame_id = 'map'
        current_time = self.get_clock().now()
        reset_time = current_time - rclpy.duration.Duration(seconds=1.0)
        local_msg.header.stamp = reset_time.to_msg()
        local_msg.pose.pose.position.x = 0.0
        local_msg.pose.pose.position.y = 0.0
        local_msg.pose.pose.position.z = 0.0
        local_msg.pose.pose.orientation.w = 1.0

        covariance = [0.0] * 36
        covariance[0] = 1e-6   # x position - high confidence but not infinite
        covariance[7] = 1e-6   # y position - high confidence but not infinite
        covariance[14] = 1e6   # z position - low confidence (let sensors handle this)
        covariance[21] = 1e6   # roll - low confidence (let IMU handle this)
        covariance[28] = 1e6   # pitch - low confidence (let IMU handle this)
        covariance[35] = 1e6   # yaw - low confidence (let IMU handle this)

        local_msg.pose.covariance = covariance

        self.local_pub.publish(local_msg)
        self.get_logger().info('Published reference local position.')

        # TODO: Add automatic checking that this worked and output it as a boolean

        return

    # Call the configuration services and wait for them to complete
    def call_service_and_wait(self, client, request, operation_name, timeout=10.0):

        self.get_logger().info(f' Starting {operation_name}...')

        # Make the service call
        future = client.call_async(request)

        # Wait for service to complete
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                self.get_logger().error(
                    f' {operation_name} timed out after {timeout} seconds')
                return False

        # Get the result
        try:
            response = future.result()
            if response and hasattr(response, 'success'):
                if response.success:
                    self.get_logger().info(
                        f' {operation_name} completed successfully: {response.message}')
                    return True
                else:
                    self.get_logger().error(
                        f' {operation_name} failed: {response.message}')
                    return False
            else:
                self.get_logger().error(f'{operation_name} returned no valid response')
                return False

        except Exception as e:
            self.get_logger().error(f'{operation_name} failed with exception: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)

    # Use MultiThreadedExecutor instead of default single-threaded
    executor = rclpy.executors.MultiThreadedExecutor()
    node = Initializer()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
