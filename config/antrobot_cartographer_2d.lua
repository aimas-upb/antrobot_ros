-- Copyright (C) 2025 Dan Novischi. All rights reserved.
-- This software may be modified and distributed under the terms of the
-- GNU Lesser General Public License v3 or any later version.

-- This file contains the configuration for Cartographer in a 2D setup. In this
-- configuration, odometry is provided externally by wheel encoders or ICP
-- (Iterative Closest Point) or a combination. Both Cartographer's SLAM
-- front-end and back-end are utilized to provide accurate mapping and
-- localization. The front-end handles sensor data processing and pose
-- estimation, including scan matching, while the back-end optimizes the pose
-- graph for loop closure. We use Cartographer for the full SLAM process here,
-- leveraging its capabilities for both scan matching and loop closure.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,  
  trajectory_builder = TRAJECTORY_BUILDER,  
  map_frame = "map",  -- The frame in which the map is published
  tracking_frame = "base_footprint",  -- The frame used for tracking the robot's pose
  published_frame = "odom",  -- The frame in which the pose is published by cartographer
  odom_frame = "odom",  -- The frame in which odometry is provided
  provide_odom_frame = false,  -- Whether Cartographer should publish the odometry frame ( we're already providing it)
  publish_frame_projected_to_2d = true,  -- Whether or not to project the published frame to 2D (yes when using 2D lidars)
  use_odometry = true,  -- Whether or not to use odometry data (yes where provinging it externally)
  use_nav_sat = false,  -- Whether or not to use GPS data
  use_landmarks = false,  -- Whether or not to use landmarks for localization
  num_laser_scans = 1,  -- Number of 2D laser scaners (we're using one 2D lida)
  num_multi_echo_laser_scans = 0,  -- Number of multi-echo laser scans (rplidar doesn't support this)
  num_subdivisions_per_laser_scan = 1,  -- Number of subdivisions per laser scan (process the full scan at onnce)
  num_point_clouds = 0,  -- Number of 3D point clouds (we're using 2D lidar)
  lookup_transform_timeout_sec = 0.2,  -- Timeout for looking up transforms
  submap_publish_period_sec = 0.3,  -- Period for publishing submaps
  pose_publish_period_sec = 5e-3,  -- Period for publishing poses
  trajectory_publish_period_sec = 30e-3,  -- Period for publishing trajectories
  rangefinder_sampling_ratio = 1.,  -- Sampling ratio for rangefinder data (use 100% of scan data)
  odometry_sampling_ratio = 1.,  -- Sampling ratio for odometry data (use 100% of odometry data)
  fixed_frame_pose_sampling_ratio = 1.,  -- Sampling ratio for fixed frame pose data (process all fixed frame poses)
  imu_sampling_ratio = 1.,  -- Sampling ratio for IMU data (use 100% of IMU data if but momentre is disabled)
  landmarks_sampling_ratio = 1.,  -- Sampling ratio for landmark data (use 100% of landmark data, not used)
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- Use the 2D trajectory builder

TRAJECTORY_BUILDER_2D.min_range = 0.12  -- Minimum range of the laser scanner
TRAJECTORY_BUILDER_2D.max_range = 3.5  -- Maximum range of the laser scanner
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.  -- Length of rays for missing data (for missing rays assume they're at a 3m distance)
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Whether or not to use IMU data
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Whether or not to use online correlative scan matching (yes aling the incomming lidar scans with the map)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- Maximum angle for the motion filter

POSE_GRAPH.constraint_builder.min_score = 0.8  -- Minimum scan matching score for constraints (bellow constriants are ignored)  
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85  -- Minimum score for global localization (bellow scores ignore scan matching for localization)
POSE_GRAPH.optimize_every_n_nodes = 10  -- Number of nodes after which to optimize the pose graph

return options
