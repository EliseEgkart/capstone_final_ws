include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
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

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 6.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 6.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

POSE_GRAPH.constraint_builder.min_score = 0.72
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.80

-- ===== 여기부터 추가 추천 =====

-- 1. submap을 조금 더 자주 끊어서 복도 재방문 시 왜곡 누적 완화
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- 2. 돌아올 때 복도 방향이 살짝 비틀리는 현상 완화
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 80.

-- 3. loop closure / pose graph 최적화를 더 자주 수행
POSE_GRAPH.optimize_every_n_nodes = 90

-- 4. 재방문 constraint 후보를 더 적극적으로 찾기
POSE_GRAPH.constraint_builder.sampling_ratio = 0.15

POSE_GRAPH.constraint_builder.max_constraint_distance = 3.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(5.)

-- 5. odom yaw를 backend에서 너무 세게 믿지 않도록 완화
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

return options