#!/usr/bin/env python3
# Copyright (C) 2021 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
import sys
import rospy
from geometry_msgs.msg import Twist
from antrobot_ros.rdrive import RDrive


class AntRobotNode:
    def __init__(self, freq=50, cmd_vel_topic='cmd_vel'):
        self.freq = freq
        self.cmd_vel_topic = cmd_vel_topic
        self.drive = RDrive()

    def __cmd_vel_callback__(self, msg: Twist) -> None:
        x = msg.linear.x
        rot = msg.angular.z
        self.drive.cmd_vel(x, rot)

    def run(self) -> None:
        # Initialize ant robot node
        rospy.init_node('ant_robot_node', anonymous=True)
        rospy.loginfo("Ant robot node started ...")

        # Enable drive
        ok = self.drive.enable()
        if not ok:
            rospy.logerr("Failed to initialize rdrive!")
            raise RuntimeError("RDrive enabled failed.")
        # TODO: Add these as params on the server
        self.drive.set_wheel_radius(0.03)
        self.drive.set_wheel_separation(0.136)
        self.drive.set_encoder_cpr(0, 1800)
        self.drive.set_encoder_cpr(1, 1800)
        self.drive.cmd_vel(0, 0)

        # Subscribe to cmd_vel messages
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.__cmd_vel_callback__)

        rospy.loginfo("Ant robot node running ...")
        # TODO: enable this when adding the tf broadcaster (don't forget to remove spin)
        # rate = rospy.Rate(self.freq)
        # while not rospy.is_shutdown():
        #     rate.sleep()
        rospy.spin()
        self.drive.cmd_vel(0, 0)
        self.drive.disable()


if __name__ == "__main__":
    args = rospy.myargv(argv=sys.argv)

    # TODO: Add args when launcher is ready
    robot_node = AntRobotNode()
    robot_node.run()
