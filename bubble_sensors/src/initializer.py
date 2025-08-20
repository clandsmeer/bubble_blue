#!/usr/bin/env python3
"""
File: initializer v1.0
Author: Henry Adam
Date: Aug 19, 2025

The purpose of this file is to complete the automated initialization of each of the sensos 
on the BlueRov2. It is called via the /initialize service of type std_srvs/srv/Trigger.  
"""

import rclpy
from rclpy.node import Node
from bubble_sensors.srv import ConfigureVN100
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

class Initializer(Node):
    def __init__(self):
        super().__init__('initializer')
        
        # Call the ConfigureVN100 service
        self.cli = self.create_client(ConfigureVN100, '/configure_vn100')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for /configure_vn100 service...')

        # Publishers for global and local positions
        self.global_pub = self.create_publisher(NavSatFix, '/mavros/global_position/global_position', 10)
        self.local_pub = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)

        # Publish reference positions after a short delay
        self.timer = self.create_timer(3.0, self.publish_reference_positions)

        #create the triggerred service that initailizes the system. 
        self.init_service = self.create_service(Trigger, "initialize", self.init_service_callback)
    
    def init_service_callback(self, request, response):
        #the purpose of this service is to initialize the imu, dvl, and ardusub ek3 output

        #first, send the imu the command to output the correct data from the correct ports 
        req_port1_dataType = ConfigureVN100.Request()
        req_port1_dataType.port = "imu"
        req_port1_dataType.msg = "VNWRG,06,16,1"#True body-fixed accelerations
        response_port1_dataType = self.cli.call_async(req_port1_dataType)
        self.get_logger().info(f'Sent configuration command for port 1 output data type. Response: {response_port1_dataType.result()}')

        req_port1_dataRate = ConfigureVN100.Request()
        req_port1_dataRate.port = "imu"
        req_port1_dataRate.msg = "VNWRG,07,50,1" #50Hz 
        response_port1_dataRate = self.cli.call_async(req_port1_dataRate)
        self.get_logger().info(f'Sent configuration command for port 1 output data rate. Response: {response_port1_dataRate.result()}')
        
        req_port2_dataType = ConfigureVN100.Request()
        req_port2_dataType.port = "imu"
        req_port2_dataType.msg = "VNWRG,06,254,2" #kalman-filterred output
        response_port2_dataType = self.cli.call_async(req_port2_dataType)
        self.get_logger().info(f'Sent configuration command for port 2 output data type. Response: {response_port2_dataType.result()}')

        req_port2_dataRate = ConfigureVN100.Request()
        req_port2_dataRate.port = "imu"
        req_port2_dataRate.msg = "VNWRG,07,50,2"
        response_port2_dataRate = self.cli.call_async(req_port2_dataRate)
        self.get_logger().info(f'Sent configuration command for port 2 output data rate. Response: {response_port2_dataRate.result()}')
        
        #finally, send the correct orientation for the imu wrt to the body frame
        req_orient = ConfigureVN100.Request()
        req_orient.port = "imu"
        req_orient.msg = "VNWRG,26,1,0,0,0,0,1,0,-1,0"
        response_orient = self.cli.call_async(req_orient)
        self.get_logger().info(f'Sent configuration command for sensor orientation. Response: {response_orient.result()}')

        # Next, send the global/local reference positions
        self.publish_reference_positions()

        # TODO: More sophisticated handling of service response 
        response.success = True
        response.message = "PLACEHOLDER MESSAGE FOR SERVICE STATUS FOR INITIAL TESITNG. MUST UPDATE."
        return response

    def publish_reference_positions(self):
        # Reference global position (example values)
        global_msg = NavSatFix()
        global_msg.latitude = 37.4275
        global_msg.longitude = -122.1697
        global_msg.altitude = 30.0
        self.global_pub.publish(global_msg)
        self.get_logger().info('Published reference global position.')

        # Reference local position (example values)
        local_msg = PoseStamped()
        local_msg.header.frame_id = "map"
        local_msg.pose.position.x = 0.0
        local_msg.pose.position.y = 0.0
        local_msg.pose.position.z = 0.0
        local_msg.pose.orientation.w = 1.0
        self.local_pub.publish(local_msg)
        self.get_logger().info('Published reference local position.')

        #TODO: Add automatic checking that this worked and output it as a boolean 

        return 

def main(args=None):
    rclpy.init(args=args)
    node = Initializer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
