
### cd ~/capstone_s2_ws/src/realsense-ros
git fetch --tags
git checkout 4.56.4

# 워크스페이스에서 realsense 관련 캐시를 확실히 제거
cd ~/capstone_s2_ws
rm -rf build/realsense2_camera build/realsense2_* install/realsense2_* log

# 다시 빌드
colcon build --packages-up-to realsense2_camera \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

ros2 launch manipulator capstone_arm_display.launch.py

ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x15 pointcloud.enable:=true align_depth.enable:=true

ros2 run camera_perception_pkg yolov8_node

ros2 run camera_perception_pkg object_distance_node --ros-args   -p detections_topic:=/detections   -p depth_image_topic:=/camera/camera/aligned_depth_to_color/image_raw   -p depth_camera_info_topic:=/camera/camera/color/camera_info   -p base_frame:=camera_link   -p target_class_name:=btn_2_deactive

ros2 run camera_perception_pkg yolov8_debug_node


moonshot@moonshot:~/open_manipulator$ ros2 topic pub /manipulator_manager/cmd std_msgs/msg/String "{data: 'status'}" --once
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='status')

moonshot@moonshot:~/open_manipulator$ ros2 topic pub /manipulator_manager/cmd std_msgs/msg/String "{data: 'go'}" --once
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='go')

moonshot@moonshot:~/capstone_s2_ws$   

moonshot@moonshot:~/capstone_s2_ws$ ros2 launch manipulator_moveit moveit_core.launch.py 
