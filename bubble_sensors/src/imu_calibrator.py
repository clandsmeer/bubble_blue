#!/usr/bin/env python3
"""
File: imu_calibrator v1.0
Author: Henry Adam
Date: Aug 18, 2025

The purpose of this file is to complete the automated calibration of the vectornav vn100 IMU. 
In the file, the raw output is 
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
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
        self.declare_parameter("imu_port", '/dev/ttyAMA3')
        self.declare_parameter('kf_port', '/dev/ttyAMA4')
        self.declare_parameter('baudrate', 115200)

        # Configure the two UART ports(imu port for imu, kf port for kalman filter outputs)
        self.imu_port = self.get_parameter("imu_port").get_parameter_value().string_value 
        self.kf_port  = self.get_parameter("kf_port").get_parameter_value().string_value  
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        # some configuration for getting the samples
        self.min_samples = 10
        self.averaging_time = 5 
        try: 
            self.ser_imu = serial.Serial(self.imu_port, self.baudrate, timeout=1)
            self.ser_kf  = serial.Serial(self.kf_port,  self.baudrate, timeout=1)
        except Exception as e: 
            self.get_logger().error(f"ERROR OPENING SERIAL PORTS: {e}")

        # Create service: when called, performs calibration and config
        self.srv = self.create_service(Trigger, 'configure_vn100', self.handle_configure)

        self.get_logger().info("VN100 Configurator ready. Call /configure_vn100 service.")

    def handle_configure(self, request, response):
        self.get_logger().info("Collecting stationary accelerometer data...")

        # Step 1: Read raw acceleration from Calibrated IMU (VNCMV) on imu_port
        accel_samples = []
        timeout = time.time() + self.averaging_time  
        while time.time() < timeout:
            line = self.ser_imu.readline().decode('ascii', errors='ignore').strip()
            if line.startswith("$VNCMV"):  
                try:
                    parts = line.split(',')
                    # VNCMV fields: magX, magY, magZ, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp, pres
                    ax, ay, az = map(float, parts[4:7])
                    accel_samples.append([ax, ay, az])
                except Exception as e:
                    self.get_logger().error(f"ERROR IN CALIBRATION: {e}")
                    continue

        if len(accel_samples) < self.min_samples:
            response.success = False
            response.message = "Not enough samples collected"
            return response

        avg_accel = np.mean(accel_samples, axis=0)
        self.get_logger().info(f"Averaged accel vector = {avg_accel}")

        # Step 2: Write gravity vector (Reg 21)
        # Leave magnetic reference as defaults (1,0,0) and set gravity = avg_accel
        mag_ref = [1.0, 0.0, 0.0]  # placeholder, you may customize later
        g = avg_accel.tolist()
        payload = f"VNWRG,21,{mag_ref[0]:.4f},{mag_ref[1]:.4f},{mag_ref[2]:.4f},{g[0]:.4f},{g[1]:.4f},{g[2]:.4f}"
        self.ser_imu.write(full_vn_message(payload))
        ack = self.ser_imu.readline().decode('ascii', errors='ignore').strip()
        self.get_logger().info(f"Set gravity vector response: {ack}")

        # Step 3: Configure async outputs
        # IMU port → ADOR=253 (VNCMV), ADOF=50 Hz
        self.ser_imu.write(full_vn_message("VNWRG,6,253"))
        self.ser_imu.write(full_vn_message("VNWRG,7,50"))

        # KF port → ADOR=254 (VNSTV), ADOF=50 Hz
        self.ser_kf.write(full_vn_message("VNWRG,6,254"))
        self.ser_kf.write(full_vn_message("VNWRG,7,50"))

        # Step 4: Save to non-volatile memory
        self.ser_imu.write(full_vn_message("VNWNV"))
        self.ser_kf.write(full_vn_message("VNWNV"))

        response.success = True
        response.message = "VN100 configured with measured gravity vector and async outputs."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VN100Configurator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
