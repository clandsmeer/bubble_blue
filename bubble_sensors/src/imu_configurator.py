#!/usr/bin/env python3
"""
File: imu_configurator.py v1.1
Author: Henry Adam
Date: Aug 19, 2025

The purpose of this file is to complete the automated calibration of the vectornav vn100 IMU. 
In the file, the raw output is 
"""

import rclpy
from rclpy.node import Node
from bubble_sensors.srv import ConfigureVN100
import serial
import time
import numpy as np

# just gets the checksum in order to complete the raw message
def vn_checksum(payload: str) -> str:
    x = 0
    for ch in payload.encode('ascii'):
        x ^= ch
    return f"{x:02X}"

# completes the vectornav message with the raw message and checksum
def full_vn_message(payload: str) -> bytes:
    cks = vn_checksum(payload)
    return f"${payload}*{cks}\r\n".encode('ascii')

class VN100Configurator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')

        # declare the parameters for the ports and baudrate
        self.declare_parameter("imu_port", '/dev/ttyAMA5')
        self.declare_parameter('kf_port', '/dev/ttyAMA4')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('print_output', False)
        self.declare_parameter('read_time', 10.0)

        # Configure the two UART ports(imu port for imu, kf port for kalman filter outputs)
        self.imu_port = self.get_parameter("imu_port").get_parameter_value().string_value 
        self.kf_port  = self.get_parameter("kf_port").get_parameter_value().string_value  
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.print_output = self.get_parameter("print_output").get_parameter_value().bool_value
        self.read_time = self.get_parameter("read_time").get_parameter_value().double_value
        # some configuration for the ports
        try: 
            self.ser_imu = serial.Serial(self.imu_port, self.baudrate, timeout=1)
            self.ser_kf  = serial.Serial(self.kf_port,  self.baudrate, timeout=1)
        except Exception as e: 
            self.get_logger().error(f"ERROR OPENING SERIAL PORTS: {e}")

        # Create service: when called, performs calibration and config
        self.srv = self.create_service(ConfigureVN100, 'configure_vn100', self.handle_configure)

        self.get_logger().info("VN100 Configurator ready. Call /configure_vn100 service.")

    def handle_configure(self, request, response):
        # parse the request 
        port = request.port
        try: 
            if port == "imu": 
                self.ser_imu.write(full_vn_message(request.msg))
            if port == "kf":
                self.ser_imu.write(full_vn_message(request.msg))
        except Exception as e: 
            self.get_logger().error(f"Error sending the request! Error: {e}") 
            response.success = False
            response.message = f"Error in sending the request: {e}"
            return response

        if self.print_outputs:
            # now read all of the messages coming in from the port
            self.get_logger().info("Reading Logs for {self.read_time} seconds")
            timeout = self.read_time + time.time()
            try: 
                while time.time() < timeout: 
                    if port == "imu": 
                        line = self.ser_imu.readline().decode('ascii', errors='ignore').strip()  
                        self.get_logger().info(f"FROM IMU PORT: {line}") 
                    if port == "kf":
                        line = self.ser_kf.readline().decode('ascii', errors='ignore').strip() 
                        self.get_logger().info(f"FROM KF PORT: {line}") 
            except KeyboardInterrupt: 
                self.get_logger().warn("KeyboardInterrupt on listening step")

        response.success = True
        response.message = f"VN100 message sent:d {str(request)}."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VN100Configurator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
