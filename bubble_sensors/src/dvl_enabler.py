#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dvl_msgs.msg import ConfigCommand

class DVLConfigurator(Node):
    def __init__(self):
        super().__init__('dvl_configurator')
        self.publisher_ = self.create_publisher(
            ConfigCommand,
            '/dvl/config/command',
            10)
        # Wait 3 seconds before sending
        self.declare_parameter('timer_wait', 3.0)
        self.timer_wait = self.get_parameter('timer_wait').get_parameter_value(
            ).double_value
        self.timer = self.create_timer(self.timer_wait, self.send_config)
        self.sent = False

    def send_config(self):
        if not self.sent:
            msg = ConfigCommand()
            msg.command = 'set_config'
            msg.parameter_name = 'acoustic_enabled'
            msg.parameter_value = 'true'
            self.publisher_.publish(msg)
            self.get_logger().info(
                f'Sent acoustic_enabled = true after {self.timer_wait} seconds')
            self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = DVLConfigurator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
