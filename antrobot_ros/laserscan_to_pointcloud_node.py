#!/usr/bin/env python3
# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import laser_geometry.laser_geometry as lg
from sensor_msgs_py.point_cloud2 import create_cloud
import tf2_ros
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy, Duration

class LaserToPointCloudNode(Node):
    def __init__(self):
        super().__init__('laser_to_pointcloud')

        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('pointcloud_topic', 'scan_pointcloud')
        self.declare_parameter('include_point_timestamp', True)

        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        pointcloud_topic = self.get_parameter('pointcloud_topic').get_parameter_value().string_value
        self.include_point_timestamp = self.get_parameter('include_point_timestamp').get_parameter_value().bool_value

        if self.include_point_timestamp:
            self.get_logger().info('Inlude point timestamp agumentation enabled.')
        else:
            self.get_logger().info('Include point timestamp agumentation disabled.')

        self.lp = lg.LaserProjection()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Define a QoS profile for LaserScan and PointCloud2 data:
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,             # Only keep the most recent message.
            depth=10,                                        # Depth of 10
            reliability=QoSReliabilityPolicy.BEST_EFFORT,    # Occasional loss is acceptable.
            durability=QoSDurabilityPolicy.VOLATILE,         # Data is transient; no need for storage.
            deadline=Duration(seconds=0),                    # No deadline enforcement.
            lifespan=Duration(seconds=0),                    # Messages do not expire.
            liveliness=QoSLivelinessPolicy.AUTOMATIC,        # Default liveliness behavior.
            liveliness_lease_duration=Duration(seconds=0)    # Liveliness lease duration not set.
        )

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, qos_profile
        )
        
        self.pc_pub = self.create_publisher(
            PointCloud2, pointcloud_topic, qos_profile
        )

        self.no_subscribers_logged = False
        self.started_publishing_logged = False

    def scan_callback(self, msg):
        try:
            if self.pc_pub.get_subscription_count() > 0:
                cloud = self._convert_scan_to_pointcloud(msg)
                self.pc_pub.publish(cloud)
                if not self.started_publishing_logged:
                    self.get_logger().info('Started publishing to PointCloud2 topic.')
                    self.started_publishing_logged = True
                self.no_subscribers_logged = False
            else:
                if not self.no_subscribers_logged:
                    self.get_logger().info('No subscribers to PointCloud2 topic, stopping publication.')
                    self.no_subscribers_logged = True
                self.started_publishing_logged = False
                return
        except Exception as e:
            self.get_logger().error(f"Error converting scan: {str(e)}")

    def _calculate_point_times(self, msg):
        point_times = []
        for i in range(len(msg.ranges)):
            point_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 + i * msg.time_increment
            point_times.append(point_time)
        return point_times

    def _convert_scan_to_pointcloud(self, msg):
        cloud = self.lp.projectLaser(msg)
        cloud.header.frame_id = msg.header.frame_id

        if self.include_point_timestamp:
            # Calculate the time for each point
            point_times = self._calculate_point_times(msg)

            # Convert the point cloud to a list of points
            cloud_data = []
            for i in range(len(cloud.data) // cloud.point_step):
                point = []
                for j in range(cloud.point_step // 4):  # Assuming each field is 4 bytes (float32)
                    point.append(struct.unpack('f', cloud.data[i * cloud.point_step + j * 4:i * cloud.point_step + (j + 1) * 4])[0])
                cloud_data.append(point)

            # Add the timestamp field to the point cloud
            for i in range(len(cloud_data)):
                if len(cloud_data[i]) == 4:  # Ensure the point has 4 fields before adding the timestamp
                    cloud_data[i].append(point_times[i])
                elif len(cloud_data[i]) == 3:  # Handle case where intensity field is missing
                    cloud_data[i].append(0.0)  # Add default intensity value
                    cloud_data[i].append(point_times[i])

            # Create a new PointCloud2 message with the specified fields structure
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='timestamp', offset=16, datatype=PointField.FLOAT32, count=1)
            ]
            cloud_with_timestamps = create_cloud(cloud.header, fields, cloud_data)

            return cloud_with_timestamps

        return cloud

def main(args=None):
    rclpy.init(args=args)
    node = LaserToPointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()