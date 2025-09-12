#!/usr/bin/env python3
"""
File: sim_vectornav_manager v1.0.

Author: Henry Adam
Date: Sept 12, 2025

The purpose of this file is to make the imu initialization services avaliable
for the simulation. It does not actually configure anything, it just receives
service calls and records successful service calls
"""


from bubble_sensors.srv import ConfigureVN100
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger


class SimVN100Manager(Node):
    def __init__(self):
        super().__init__('sim_vectornav_manager')

        # Create service for configuring vectornav registers
        self.config_srv = self.create_service(ConfigureVN100,
                                              'configure_vn100',
                                              self.handle_configure)

        # Create service for Triggering Bias estimation process
        self.bias_srv = self.create_service(Trigger,
                                            'estimate_bias',
                                            self.estimate_biases)

        # Create service for starting and stopping the serial port process
        self.read_srv = self.create_service(SetBool,
                                            'set_reading_status',
                                            self.toggle_reading_status)

        self.get_logger().info(
            'VN100 Command Node is Running. Use the /ConfigureVN100 service call '
            'to send a message to a particular port'
        )

    def handle_configure(self, request, response):
        # parse the request
        self.get_logger().info(
            f'Simulated sending message {request.msg} to port {request.port}'
        )

        response.success = True
        response.message = f'VN100 message sent:d {str(request)}.'
        self.get_logger().info('Handle Port Finished Correctly. Sending Response')
        return response

    def estimate_biases(self, request, response):
        self.get_logger().info(
            'Simulated bias estimation. Sending Successful message'
        )
        response.success = True
        response.message = 'VN100 bias estimated.'
        self.get_logger().info('Handle Port Finished Correctly. Sending Response')
        return response
    
    def toggle_reading_status(self, request, response):
        self.get_logger().info('Simulation received Command to Toggle Reading Status to {request.data}')
        response.success = True
        response.message = f'Updated Publishing Data to {request.data}'
        return response 
    
def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.SingleThreadedExecutor()
    node = SimVN100Manager()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...(hopefully)')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
