#!/usr/bin/env python3
# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from antrobot_ros.rdrive import RDrive
from geometry_msgs.msg import Twist


class RDriveNode(Node):
    def __init__(self):
        super().__init__('rdrive_node')
                
        # Declare parameters with default values (in case the antrobot_param.yaml is somehow missing)
        self.declare_parameter(
            'wheel_radius', 
            0.03, 
            ParameterDescriptor(description='Radius of the wheels')
        )
        self.declare_parameter(
            'wheel_separation', 
            0.219, 
            ParameterDescriptor(description='Separation between the wheels')
        )
        self.declare_parameter(
            'encoder_cpr_left', 
            2940, 
            ParameterDescriptor(description='Encoder counts per revolution for the left wheel')
        )
        self.declare_parameter(
            'encoder_cpr_right', 
            2940, 
            ParameterDescriptor(description='Encoder counts per revolution for the right wheel')
        )
        
        
        # Get the config rdrive parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.encoder_cpr_left = self.get_parameter('encoder_cpr_left').value
        self.encoder_cpr_right = self.get_parameter('encoder_cpr_right').value
        
        # Define a QoS profile for command velocity
        cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=Duration(seconds=0),
            lifespan=Duration(seconds=0),
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=0)
        )
        
        # Create the velocity command subscription
        self.cmd_vel_subscriber = self.create_subscription(
            msg_type=Twist, 
            topic='cmd_vel',
            callback=self.__cmd_vel_callback, 
            qos_profile=cmd_vel_qos
        )
        
        # Set node internal rdrive state
        self.drive_state = False
        
        # Intaintate RDrive
        self.drive = RDrive()    

    def __cmd_vel_callback(self, msg: Twist) -> None:
        v = msg.linear.x
        omega = msg.angular.z
        self.drive.cmd_vel(v, omega)
    
    def drive_init(self) -> None:
        self.get_logger().info("RDrive initializing...")
        self.drive_state = self.drive.enable()
        if self.drive_state:
            self.drive.set_wheel_radius(self.wheel_radius)
            self.get_logger().info('Set RDrive wheel radius: "%f"' % self.wheel_radius)
            
            self.drive.set_wheel_separation(self.wheel_separation)
            self.get_logger().info('Set RDrive wheel separation: "%f"' % self.wheel_separation)
            
            self.drive.set_encoder_cpr(0, self.encoder_cpr_left)
            self.get_logger().info('Set RDrive encoder CPR left: "%d"' % self.encoder_cpr_left)
            
            self.drive.set_encoder_cpr(1, self.encoder_cpr_right)
            self.get_logger().info('Set RDrive encoder CPR right: "%d"' % self.encoder_cpr_right)            
            
            self.drive.cmd_vel(0, 0)
            self.get_logger().info('RDrive running ...')
        else:
            self.get_logger().error('RDrive failed to initialize!')
    
    def get_drive_state(self) -> bool:
        return self.drive_state

    def drive_shutdown(self) -> None:
        if self.drive_state:
            # Stop any rdrive command
            self.drive.cmd_vel(0, 0)
            
            # Disable rdrive
            self.drive_state = self.drive.disable() 
            
            if not self.drive_state:
                self.get_logger().info('RDrive shutdown.')
    
    def destroy_node(self):
        self.drive_shutdown()
        return super().destroy_node()
            
        
def main(args=None):
    # Initialize ros client lib
    rclpy.init(args=args)
    
    # Intatiate the rdrive node
    rdirve_node = RDriveNode()
    
    # Intialize rdrive
    rdirve_node.drive_init()
    
    # If intialization failed
    if not rdirve_node.get_drive_state():
        rdirve_node.destroy_node()
        rclpy.shutdown()
        return
    try:
        # Run rdrive node
        rclpy.spin(rdirve_node)
    except KeyboardInterrupt:
        rdirve_node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        # Don't wait for the garbage collector, free resources now!
        rdirve_node.destroy_node()
    
    # Shutdown ros client lib
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()



