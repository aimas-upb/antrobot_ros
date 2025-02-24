#!/usr/bin/env python3
# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration
# TODO: Add intial pose service for this node
class JointStateEstimator(Node):
    def __init__(self):
        super().__init__('joint_state_estimator')

        # Define a QoS profile for joint state data:
        joint_state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,             # Only keep the most recent message.
            depth=5,                                         # Depth of 5 (note only the latest update is needed)
            reliability=QoSReliabilityPolicy.BEST_EFFORT,    # Occasional loss is acceptable.
            durability=QoSDurabilityPolicy.VOLATILE,         # Data is transient; no need for storage.
            deadline=Duration(seconds=0),                    # No deadline enforcement.
            lifespan=Duration(seconds=0),                    # Messages do not expire.
            liveliness=QoSLivelinessPolicy.AUTOMATIC,        # Default liveliness behavior.
            liveliness_lease_duration=Duration(seconds=0)    # Liveliness lease duration not set.
        )

        # Declare parameters
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation', 0.219)
        self.declare_parameter('publish_frequency', 10.0)  # Default frequency is 10 Hz

        # Get parameters
        self.odom_topic = self.get_parameter('odom_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        # Initial joint state
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
        # Subscriber to ICP-based odometry
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, joint_state_qos)

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, self.joint_state_topic, joint_state_qos)

        # Store last time and pose for velocity calculation (TODO: add service to set the intial pose)
        self.last_time = self.get_clock().now()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0

        # Timer to periodically publish joint states
        self.timer_period = 1.0 / self.publish_frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # Publish joint states periodically
        self.publish_joint_states(self.left_wheel_pos, self.right_wheel_pos, self.left_wheel_vel, self.right_wheel_vel)

    def odom_callback(self, msg):
        # Get pose from KISS-ICP
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.yaw_from_quaternion(msg.pose.pose.orientation)

        # Get current time
        current_time = self.get_clock().now()

        # Calculate time difference
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds

        if dt > 0:
            # Estimate velocities
            v = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2) / dt
            omega = (theta - self.last_theta) / dt

            # Compute wheel positions
            self.left_wheel_pos = (x / self.wheel_radius) - ((theta * self.wheel_separation) / (2 * self.wheel_radius))
            self.right_wheel_pos = (x / self.wheel_radius) + ((theta * self.wheel_separation) / (2 * self.wheel_radius))

            # Compute velocities
            self.left_wheel_vel = (v / self.wheel_radius) - ((omega * self.wheel_separation) / (2 * self.wheel_radius))
            self.right_wheel_vel = (v / self.wheel_radius) + ((omega * self.wheel_separation) / (2 * self.wheel_radius))

        # Update last pose and time
        self.last_x = x
        self.last_y = y
        self.last_theta = theta
        self.last_time = current_time

    def yaw_from_quaternion(self, q):
        """Extract yaw (rotation about Z) from quaternion"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_joint_states(self, left_pos, right_pos, left_vel, right_vel):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_left_joint', 'wheel_right_joint'] # TODO: provide through urdf
        msg.position = [left_pos, right_pos]
        msg.velocity = [left_vel, right_vel]
        msg.effort = []

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
