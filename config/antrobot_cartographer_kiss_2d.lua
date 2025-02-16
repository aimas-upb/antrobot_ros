-- Copyright (C) 2025 Dan Novischi. All rights reserved.
-- This software may be modified and distributed under the terms of the
-- GNU Lesser General Public License v3 or any later version.

-- This file contains the configuration for Cartographer in a 2D setup. In this
-- configuration, odometry and scan matching are provided externally by KISS-ICP
-- (Iterative Closest Point). Cartographer's SLAM back-end is utilized to provide
-- accurate loop closure. The front-end handles sensor data processing and pose
-- estimation, while the back-end optimizes the pose graph for loop closure.
-- We use Cartographer for the loop closure process here, leveraging its
-- capabilities for pose graph optimization.

-- TODO: This is not tested yet, it's a work in progress

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,  -- Configuration for the map builder
  trajectory_builder = TRAJECTORY_BUILDER,  -- Configuration for the trajectory builder
  map_frame = "map",  -- The frame in which the map is published
  tracking_frame = "base_footprint",  -- The frame used for tracking the robot's pose
  published_frame = "odom",  -- The frame in which the pose is published
  odom_frame = "odom",  -- The frame in which odometry is provided
  provide_odom_frame = false,  -- Whether Cartographer should publish the odometry frame
  publish_frame_projected_to_2d = true,  -- Whether to publish the frame projected to 2D
  use_odometry = true,  -- Whether to use odometry data
  use_nav_sat = false,  -- Whether to use navigation satellite data
  use_landmarks = false,  -- Whether to use landmarks for localization
  num_laser_scans = 1,  -- Number of laser scans
  num_multi_echo_laser_scans = 0,  -- Number of multi-echo laser scans
  num_subdivisions_per_laser_scan = 1,  -- Number of subdivisions per laser scan
  num_point_clouds = 0,  -- Number of point clouds
  lookup_transform_timeout_sec = 0.2,  -- Timeout for looking up transforms
  submap_publish_period_sec = 0.3,  -- Period for publishing submaps
  pose_publish_period_sec = 5e-3,  -- Period for publishing poses
  trajectory_publish_period_sec = 30e-3,  -- Period for publishing trajectories
  rangefinder_sampling_ratio = 1.,  -- Sampling ratio for rangefinder data
  odometry_sampling_ratio = 1.,  -- Sampling ratio for odometry data
  fixed_frame_pose_sampling_ratio = 1.,  -- Sampling ratio for fixed frame pose data
  imu_sampling_ratio = 1.,  -- Sampling ratio for IMU data
  landmarks_sampling_ratio = 1.,  -- Sampling ratio for landmarks data
}

MAP_BUILDER.use_trajectory_builder_2d = true  -- Use the 2D trajectory builder

TRAJECTORY_BUILDER_2D.min_range = 0.12  -- Minimum range for the rangefinder
TRAJECTORY_BUILDER_2D.max_range = 3.5  -- Maximum range for the rangefinder
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.  -- Length of rays for missing data
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- Whether to use IMU data
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false  -- Whether to use online correlative scan matching
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50  -- Number of range data points per submap
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- Maximum angle for the motion filter

POSE_GRAPH.constraint_builder.min_score = 0.8  -- Minimum score for constraints
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.85  -- Minimum score for global localization
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5  -- Sampling ratio for constraints
POSE_GRAPH.optimize_every_n_nodes = 5  -- Number of nodes after which to optimize the pose graph

return options
