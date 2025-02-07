#!/usr/bin/env python3
# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rdrive import RDrive

class RDriveNode(Node):
    def __init__(self):
        super().__init__('rdrive')
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.__cmd_vel_callback, 10
        )
        self.drive = RDrive()

    def __cmd_vel_callback(self, msg: Twist) -> None:
        v = msg.linear.x
        omega = msg.linear.z
        self.drive.cmd_vel(v, omega)
    
    def init(self) -> None:
        self.get_logger().info("AR: initializing")
        
        self.drive.set_wheel_radius(0.03)
        self.drive.set_wheel_separation(0.1345)
        self.drive.set_encoder_cpr(0, 2100)
        self.drive.set_encoder_cpr(1, 2100)
        self.drive.cmd_vel(0, 0)

    
    def run(self) -> None:
        ok = self.drive.enable()
    
    def shutdown(self) -> None:
        self.drive.disable()     
        
def main(args=None):
    rclpy.init(args=args)
    rdirve_node = RDriveNode()
    rclpy.spin(rdirve_node)
    rdirve_node.destroy_node()
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()

   

