#!/usr/bin/env python3
"""
File: vectornav_manager v2.0
Author: Henry Adam
Date: Aug 18, 2025

The purpose of this file is to complete the automated calibration of the vectornav vn100 IMU. 
In the file, the raw output is 
"""

import rclpy
from rclpy.node import Node
from bubble_sensors.srv import ConfigureVN100
import serial
import time
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading

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

class VN100Manager(Node):
    def __init__(self):
        super().__init__('vectornav_manager')

        # declare the parameters for the ports and baudrate
        self.declare_parameter("port1", '/dev/ttyAMA4')
        self.declare_parameter('port2', '/dev/ttyAMA5')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('print_output', False)
        self.declare_parameter('read_time', 10.0)

        # Configure the two UART ports(imu port for imu, kf port for kalman filter outputs)
        self.port1 = self.get_parameter("port1").get_parameter_value().string_value 
        self.port2  = self.get_parameter("port2").get_parameter_value().string_value  
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.print_output = self.get_parameter("print_output").get_parameter_value().bool_value
        self.read_time = self.get_parameter("read_time")
        # some configuration for the ports
        try: 
            self.ser_port1 = serial.Serial(self.port1, self.baudrate, timeout=1, rtscts=True)
            self.ser_port2  = serial.Serial(self.port2,  self.baudrate, timeout=1, rtscts=True)
        except Exception as e: 
            self.get_logger().error(f"ERROR OPENING SERIAL PORTS: {e}")

        # Create service: when called, performs calibration and config
        self.srv = self.create_service(ConfigureVN100, 'configure_vn100', self.handle_configure)

        self.get_logger().info("VN100 Command Node is Running. Use the /ConfigureVN100 service call to send a message to a particular port")
        self.get_logger().info(f"The imu port is {self.port1}, and the kf port is {self.port2}")
        self.get_logger().info("Example Command: ros2 service call /configure_vn100 bubble_sensors/srv/ConfigureVN100 \"{port:'imu', msg:'VNWRG,06,1,1'} ")

        #Create the publishers for the ros nodes
        self.imu_pub = self.create_publisher(Imu, "/vectornav/Imu_body", 10)
        self.orient_pub = self.create_publisher(PoseWithCovarianceStamped, "/vectornav/filterred_orientation", 10)

        self.reading = False
        self.port1_thread = None
        self.port2_thread = None
        self.thread_lock = threading.Lock()
    
    def port1_reader(self): 
        while self.reading: 
            try: 
                msg = self.ser_port1.readline().decode('ascii', errors='ignore').strip()  
                if not msg:
                    #empty messages, skip this
                    continue
                parsed_msg = msg.split("*")[0].split(",")
                msg_type = parsed_msg[0]
                imu_message = Imu()
                # TODO: Add message type handling
                # For now, assuming port 1 has gravity-compensated accel data
                if msg_type == "$VNYBA":
                    yaw = float(parsed_msg[1])
                    cy = np.cos(yaw * np.pi/180 * 0.5)
                    sy = np.sin(yaw * np.pi/180 * 0.5)
                    
                    pitch = float(parsed_msg[2])
                    cp = np.cos(pitch * np.pi/180 * 0.5)
                    sp = np.sin(pitch * np.pi/180 * 0.5)

                    roll = float(parsed_msg[3])
                    cr = np.cos(roll * np.pi/180 * 0.5)
                    sr = np.sin(roll * np.pi/180 * 0.5)

                    #Orientation and Associated Covariance
                    imu_message.orientation.x = sr * cp * cy - cr * sp * sy
                    imu_message.orientation.y = cr * sp * cy + sr * cp * sy
                    imu_message.orientation.z = cr * cp * sy - sr * sp * cy
                    imu_message.orientation.w = cr * cp * cy + sr * sp * sy
                    imu_message.orientation_covariance[0] = 0.02 #to make it higher than KF-output
                    imu_message.orientation_covariance[4] = 0.02
                    imu_message.orientation_covariance[8] = 0.02

                    #Angular Velocity and Associated Covariance
                    imu_message.angular_velocity.x = float(parsed_msg[7])
                    imu_message.angular_velocity.y = float(parsed_msg[8])
                    imu_message.angular_velocity.z = float(parsed_msg[9])
                    imu_message.angular_velocity_covariance[0] = 0.01
                    imu_message.angular_velocity_covariance[4] = 0.01
                    imu_message.angular_velocity_covariance[8] = 0.01

                    #Linear Acceleration and Associated Covariance
                    imu_message.linear_acceleration.x = float(parsed_msg[4])
                    imu_message.linear_acceleration.y = float(parsed_msg[5])
                    imu_message.linear_acceleration.z = float(parsed_msg[6])
                    imu_message.linear_acceleration_covariance[0] = 0.01
                    imu_message.linear_acceleration_covariance[4] = 0.01
                    imu_message.linear_acceleration_covariance[8] = 0.01

                    # TODO: Make the Covariance a ros2 parameter
                    #create the header for the message
                    imu_message.header.frame_id = "base_link_frd"
                    imu_message.header.stamp = self.get_clock().now().to_msg()

                    #actually publish the message
                    self.imu_pub.publish(imu_message)
                elif msg_type == "$VNERR": 
                    # there is some error in the vn100, print it to the logger
                    self.get_logger.error(f"Error code {parsed_msg[1]} in vn100 node")
                else: 
                    #for now, there is no handling of other message types, so just throw an error here:
                    self.get_logger().error("UNHANDLED MESSAGE TYPE IN PORT 1")
            except Exception as e: 
                    self.get_logger().error(f"Error in Port 1: {e}")
        self.get_logger().info("Serial Port 1 Closing")
        return
    
    def port2_reader(self): 
        while self.reading: 
            try:
                msg = self.ser_port2.readline().decode('ascii', errors='ignore').strip()  
                if not msg:
                    #empty messages
                    continue
                parsed_msg = msg.split("*")[0].split(",")
                msg_type = parsed_msg[0]
                ekf_message = PoseWithCovarianceStamped()
                # TODO: Add message type handling
                # For now, assuming port 2 has Kalman-Filterred Data
                if msg_type == "$VNSTV":
                    
                    ekf_message.pose.pose.orientation.x = float(parsed_msg[1])
                    ekf_message.pose.pose.orientation.y = float(parsed_msg[2])
                    ekf_message.pose.pose.orientation.z = float(parsed_msg[3])
                    ekf_message.pose.pose.orientation.w = float(parsed_msg[4])
                    #for pose.pose.position, indicate that they are not to be included
                    ekf_message.pose.covariance[0] = -1
                    ekf_message.pose.covariance[7] = -1
                    ekf_message.pose.covariance[14] = -1

                    #For orientation, give example covariance
                    # TODO: Replace these with periodically sampled covariances from the vn100
                    ekf_message.pose.covariance[21] = 0.01
                    ekf_message.pose.covariance[28] = 0.01
                    ekf_message.pose.covariance[35] = 0.01

                    #create the header for the message
                    ekf_message.header.frame_id = "map_ned" #publishes the 
                    ekf_message.header.stamp = self.get_clock().now().to_msg()

                    #actually publish the message
                    self.orient_pub.publish(ekf_message)
                elif msg_type == "$VNERR": 
                    # there is some error in the vn100, print it to the logger
                    self.get_logger.error(f"Error code {parsed_msg[1]} in vn100 node")
                else: 
                    #for now, there is no handling of other message types, so just throw an error here:
                    self.get_logger().error("UNHANDLED MESSAGE TYPE IN PORT 2")
            except Exception as e: 
                    self.get_logger().error(f"Error in Port 2: {e}")
        self.get_logger().info("Serial Port 2 Closing")
        return
    
    def handle_configure(self, request, response):
        # parse the request 
        port = request.port
        try: 
            if port == "imu": 
                self.ser_port1.write(full_vn_message(request.msg))
            if port == "kf":
                self.ser_port2.write(full_vn_message(request.msg))
        except Exception as e: 
            self.get_logger().error(f"Error sending the request! Error: {e}") 
            response.success = False
            response.message = f"Error in sending the request: {e}"
            return response

        if self.print_output:
            # now read all of the messages coming in from the port
            self.get_logger().info("Reading Logs for {self.read_time} seconds")
            timeout = self.read_time + time.time()
            try: 
                while time.time() < timeout: 
                    if port == "imu": 
                        line = self.ser_port1.readline().decode('ascii', errors='ignore').strip()  
                        self.get_logger().info(f"FROM IMU PORT: {line}") 
                    if port == "kf":
                        line = self.ser_port2.readline().decode('ascii', errors='ignore').strip() 
                        self.get_logger().info(f"FROM KF PORT: {line}") 
            except KeyboardInterrupt: 
                self.get_logger().warn("KeyboardInterrupt on listening step")

        # After configuring port outputs, start the reading services for both ports
        self.start_reading_threads()
        
        response.success = True
        response.message = f"VN100 message sent:d {str(request)}."
        return response
    
    def start_reading_threads(self):
        #Bring up both threads
        with self.thread_lock:
            if not self.reading:
                self.reading = True
                
                if self.port1_thread is None or not self.port1_thread.is_alive():
                    self.port1_thread = threading.Thread(target=self.port1_reader, daemon=True)
                    self.port1_thread.start()
                    self.get_logger().info("Started Port 1 reading thread")
                
                if self.port2_thread is None or not self.port2_thread.is_alive():
                    self.port2_thread = threading.Thread(target=self.port2_reader, daemon=True)
                    self.port2_thread.start()
                    self.get_logger().info("Started Port 2 reading thread")

    def stop_reading_threads(self):
        #Bring down both threads
        with self.thread_lock:
            if self.reading:
                self.reading = False
                
                if self.port1_thread and self.port1_thread.is_alive():
                    self.port1_thread.join(timeout=2.0)
                
                if self.port2_thread and self.port2_thread.is_alive():
                    self.port2_thread.join(timeout=2.0)

def main(args=None):
    rclpy.init(args=args)
    
    executor = rclpy.executors.SingleThreadedExecutor()
    node = VN100Manager()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully...(hopefully)")
    finally:
        node.stop_reading_threads()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
