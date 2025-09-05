#!/usr/bin/env python3
"""
File: vectornav_manager v2.1
Author: Henry Adam
Date: Aug 18, 2025

The purpose of this file is to complete the automated calibration of the vectornav vn100 IMU.
In the file, the raw output is
"""

import threading
import time

import numpy as np
import rclpy
import serial
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, Trigger

from bubble_sensors.srv import ConfigureVN100


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

#Average Update Equation that only needs previous average and number of samples
def average_update(avg_k_minus_1, k, sample_k):
    return avg_k_minus_1 * ((k-1)/k) + sample_k/k

class VN100Manager(Node):
    def __init__(self):
        super().__init__('vectornav_manager')

        # declare the parameters for the ports and baudrate
        self.declare_parameter("port1", '/dev/ttyAMA4')
        self.declare_parameter('port2', '/dev/ttyAMA5')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('print_output', False)
        self.declare_parameter('read_time', 10.0)
        self.declare_parameter('imu_accel_cov', 0.05)
        self.declare_parameter('imu_ang_vel_covariance', 0.01)
        self.declare_parameter('imu_orientation_cov', 0.05)
        self.declare_parameter('init_kf_orient_cov', 0.01)
        self.declare_parameter('bias_averaging_time', 30.0)

        # Configure the two UART ports(imu port for imu, kf port for kalman filter outputs)
        self.port1 = self.get_parameter("port1").get_parameter_value().string_value
        self.port2  = self.get_parameter("port2").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.print_output = self.get_parameter("print_output").get_parameter_value().bool_value
        self.read_time = self.get_parameter("read_time").get_parameter_value().double_value
        self.imu_accel_cov = self.get_parameter("imu_accel_cov").get_parameter_value().double_value
        self.imu_ang_vel_covariance = self.get_parameter("imu_ang_vel_covariance").get_parameter_value().double_value
        self.imu_orientation_cov = self.get_parameter("imu_orientation_cov").get_parameter_value().double_value
        self.init_orient = self.get_parameter("init_kf_orient_cov").get_parameter_value().double_value
        self.orient_x_cov = self.init_orient
        self.orient_y_cov = self.init_orient
        self.orient_z_cov = self.init_orient

        # Create the variables to store IMU biases
        # bias_body_i variable stores rolling average
        self.bias_body_x = 0
        self.bias_body_y = 0
        self.bias_body_z = 0
        self.k = 1 # for rolling average
        self.bias_averaging_time = self.get_parameter("bias_averaging_time").get_parameter_value().double_value

        # some configuration for the ports
        try:
            self.ser_port1 = serial.Serial(self.port1, self.baudrate, timeout=1, rtscts=True)
            self.ser_port2  = serial.Serial(self.port2,  self.baudrate, timeout=1, rtscts=True)
        except Exception as e:
            self.get_logger().error(f"ERROR OPENING SERIAL PORTS: {e}")

        # Create service for configuring vectornav registers
        self.config_srv = self.create_service(ConfigureVN100, 'configure_vn100', self.handle_configure)

        # Create service for Triggering Bias estimation process
        self.bias_srv = self.create_service(Trigger, "estimate_bias", self.estimate_biases)
        self.estimating_bias = False

        #Create service for starting and stopping the serial port process
        self.read_srv = self.create_service(SetBool, "set_reading_status", self.toggle_reading_status)

        self.get_logger().info("VN100 Command Node is Running. Use the /ConfigureVN100 service call to send a message to a particular port")
        self.get_logger().info(f"The imu port is {self.port1}, and the kf port is {self.port2}")
        self.get_logger().info("Example Command: ros2 service call /configure_vn100 bubble_sensors/srv/ConfigureVN100 \"{port:'imu', msg:'VNWRG,06,1,1'} ")

        # Create the publishers for the ros nodes
        self.imu_pub = self.create_publisher(Imu, "/vectornav/Imu_body", 10)
        self.orient_pub = self.create_publisher(PoseWithCovarianceStamped, "/vectornav/filterred_orientation", 10)

        # Variables defining port status
        self.publishing_data = False
        self.ports_active = False
        self.port1_thread = None
        self.port2_thread = None
        self.thread_lock = threading.Lock()

        #timer for requesting the covariance matrix
        self.create_timer(1, self.request_covariance)

        #Start the reading threads at the beginning
        self.start_reading_threads()
        self.get_logger().info("Started port1 and port2 reading threads")

    def port1_reader(self):
        while self.ports_active:
            try:
                msg = self.ser_port1.readline().decode('ascii', errors='ignore').strip()
                if self.publishing_data:
                    if not msg:
                        #empty messages, skip this
                        continue
                    parsed_msg = msg.split("*")[0].split(",")
                    msg_type = parsed_msg[0]
                    imu_message = Imu()
                    # TODO: Add more message type handling
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
                        imu_message.orientation_covariance[0] = self.imu_orientation_cov #to make it higher than KF-output
                        imu_message.orientation_covariance[4] = self.imu_orientation_cov
                        imu_message.orientation_covariance[8] = self.imu_orientation_cov

                        #Angular Velocity and Associated Covariance
                        #should be compensated for bias according to ICD
                        imu_message.angular_velocity.x = float(parsed_msg[7])
                        imu_message.angular_velocity.y = float(parsed_msg[8])
                        imu_message.angular_velocity.z = float(parsed_msg[9])
                        imu_message.angular_velocity_covariance[0] = self.imu_ang_vel_covariance
                        imu_message.angular_velocity_covariance[4] = self.imu_ang_vel_covariance
                        imu_message.angular_velocity_covariance[8] = self.imu_ang_vel_covariance

                        #Linear Acceleration and Associated Covariance
                        imu_message.linear_acceleration.x = float(parsed_msg[4]) - self.bias_body_x
                        imu_message.linear_acceleration.y = float(parsed_msg[5]) - self.bias_body_y
                        imu_message.linear_acceleration.z = float(parsed_msg[6]) - self.bias_body_z
                        imu_message.linear_acceleration_covariance[0] = self.imu_accel_cov
                        imu_message.linear_acceleration_covariance[4] = self.imu_accel_cov
                        imu_message.linear_acceleration_covariance[8] = self.imu_accel_cov

                        #create the header for the message
                        imu_message.header.frame_id = "base_link_frd"
                        imu_message.header.stamp = self.get_clock().now().to_msg()

                        #actually publish the message
                        self.imu_pub.publish(imu_message)

                    elif msg_type == "$VNERR":
                        # there is some error in the vn100, print it to the logger
                        self.get_logger().error(f"Error code {parsed_msg[1]} in vn100 node")

                    elif msg_type == "$VNYMR":
                        # This is the default, so don't worry unless the port expects to be
                        #publishing data
                        if self.publishing_data:
                            self.get_logger().warn("Port 2 should be publishing data, but output is still default")

                    else:
                        #for now, there is no handling of other message types, so just throw an error here:
                        self.get_logger().error("UNHANDLED MESSAGE TYPE IN PORT 1")

                elif self.estimating_bias:
                    if not msg:
                        #empty messages, skip this
                        continue
                    parsed_msg = msg.split("*")[0].split(",")
                    msg_type = parsed_msg[0]
                    imu_message = Imu()
                    # Read the body-fixed accelerations to estimate the bias
                    if msg_type == "$VNYBA":

                        #Linear Acceleration and Associated Covariance
                        x = float(parsed_msg[4])
                        y = float(parsed_msg[5])
                        z = float(parsed_msg[6])

                        self.bias_body_x = average_update(self.bias_body_x, self.k, x)
                        self.bias_body_y = average_update(self.bias_body_y, self.k, y)
                        self.bias_body_z = average_update(self.bias_body_z, self.k, z)
                        self.k+=1
                    else:
                        self.get_logger().warn("BIAS ESTIMATE CALLED BEFORE IMU OUTPUTTING BODY-FIXED COORDINATES")

            except Exception as e:
                    self.get_logger().error(f"Error in Port 1: {e}")

        self.get_logger().info("Serial Port 1 Closing")
        return

    def port2_reader(self):
        while self.ports_active:
            try:
                msg = self.ser_port2.readline().decode('ascii', errors='ignore').strip()
                if self.publishing_data:
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

                        # For orientation, give example covariance
                        # TODO: Replace these with periodically sampled covariances from the vn100
                        ekf_message.pose.covariance[21] = self.orient_x_cov
                        ekf_message.pose.covariance[28] = self.orient_y_cov
                        ekf_message.pose.covariance[35] = self.orient_z_cov


                        # for pose.pose.position, indicate that they are not to be included
                        ekf_message.pose.covariance[0] = -1
                        ekf_message.pose.covariance[7] = -1
                        ekf_message.pose.covariance[14] = -1

                        # create the header for the message
                        ekf_message.header.frame_id = "map_ned" #publishes the
                        ekf_message.header.stamp = self.get_clock().now().to_msg()

                        # actually publish the message
                        self.orient_pub.publish(ekf_message)

                    elif msg_type == "$VNERR":
                        # there is some error in the vn100, print it to the logger
                        self.get_logger().error(f"Error code {parsed_msg[1]} in vn100 node")

                    elif msg_type == "$VNCOV":
                        self.orient_x_cov = float(parsed_msg[1])
                        self.orient_y_cov = float(parsed_msg[2])
                        self.orient_z_cov = float(parsed_msg[3])
                        self.get_logger().info(f"Orientation covariance is [{self.orient_x_cov}, {self.orient_y_cov}, {self.orient_z_cov}]")

                    elif msg_type == "$VNRRG" and int(parsed_msg[1]) == 254:
                        # this is the read register response for the covariance, read it the same as above
                        self.orient_x_cov = float(parsed_msg[2])
                        self.orient_y_cov = float(parsed_msg[3])
                        self.orient_z_cov = float(parsed_msg[4])

                    elif msg_type == "$VNYMR":
                        # This is the default, so don't worry unless the port expects to be
                        #publishing data
                        if self.publishing_data:
                            self.get_logger().warn("Port 2 should be publishing data, but output is still default")

                    else:
                        #for now, there is no handling of other message types, so just throw an error here:
                        self.get_logger().error("UNHANDLED MESSAGE TYPE IN PORT 2")
            except Exception as e:
                    self.get_logger().error(f"Error in Port 2: {e}")
        self.get_logger().info("Serial Port 2 Closing")
        return

    def handle_configure(self, request, response):
        # parse the request
        self.get_logger().info(f"Sending message {request.msg} to port {request.port}")
        port = request.port
        try:
            if port == "port1":
                self.ser_port1.write(full_vn_message(request.msg))
            elif port == "port2":
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

        response.success = True
        response.message = f"VN100 message sent:d {str(request)}."
        self.get_logger().info("Handle Port Finished Correctly. Sending Response")
        return response

    def request_covariance(self):
        msg = full_vn_message("VNRRG,254")
        self.ser_port2.write(msg)
        return

    def start_reading_threads(self):
        #Bring up both threads
        with self.thread_lock:
            if not self.ports_active:
                self.ports_active = True

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
            if self.ports_active:
                self.ports_active = False

                if self.port1_thread and self.port1_thread.is_alive():
                    self.port1_thread.join(timeout=2.0)

                if self.port2_thread and self.port2_thread.is_alive():
                    self.port2_thread.join(timeout=2.0)

    def estimate_biases(self, request, response):
        self.get_logger().info("Received Command to Estimate Bias")
        self.estimating_bias = True
        start_time = time.time()
        while time.time() - start_time < self.bias_averaging_time:
            continue
        self.estimating_bias = False
        response.success = True
        response.message = f"Bias Estimate Set. x bias = {self.bias_body_x},  y bias = {self.bias_body_y},  z bias = {self.bias_body_z}"
        return response

    def toggle_reading_status(self, request, response):
        self.get_logger().info("Received Command to Toggle Reading Status")
        if request.data:
            self.publishing_data = True
        else:
            self.publishing_data = False
        response.success = True
        response.message = f"Updated Publishing Data to {request.data}"
        return response

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
