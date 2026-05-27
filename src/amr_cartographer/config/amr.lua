include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",

  -- EKF가 odom -> base_link TF를 발행하므로
  -- Cartographer는 map -> odom을 발행하는 구조로 둠
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,

  publish_frame_projected_to_2d = true,

  -- EKF /odom 사용
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  -- baseline은 반드시 1개 LiDAR부터
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- 실내 2D LiDAR baseline
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 6.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 6.

-- IMU는 EKF에서만 사용하고, Cartographer에는 직접 넣지 않음
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- TurtleBot/실내 2D Cartographer 계열에서 흔히 쓰는 방향
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false

-- 너무 많은 튜닝 대신 기본적인 motion filter만 둠
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.07
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)

-- submap은 기본 계열에 가깝게
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- 복도에서 yaw가 매 scan마다 흔들리지 않도록 회전 prior 강화
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 250.

-- TurtleBot 계열에서 자주 쓰는 loop closure 기준
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70

return options
