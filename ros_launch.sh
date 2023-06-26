#!/bin/bash

xterm -iconic -hold -e "roscore" &
sleep 1; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; roslaunch robot.launch" & #RosAria
sleep 1; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; roslaunch hokuyo.launch" &
sleep 2; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; rosrun robot_setup_tf tf_broadcaster" &
sleep 2; xterm -iconic -hold -e "cd ~/ExperimentPlatform/; python map.py" &
sleep 2; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; rosrun rviz rviz" &
sleep 2; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; rosrun map_server map_server ~/ExperimentPlatform/md2_new.yaml" &
sleep 2; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; roslaunch AMCL.launch" &
sleep 2; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; rosrun joy joy_node _autorepeat_rate:=10"
#sleep 1; xterm -iconic -hold -e "cd ~/ExperimentPlatform/; python combined_joystick.py" &
#sleep 1; xterm -iconic -hold -e "cd ~/ExperimentPlatform/; python keyboard_teleop.py" 
# sleep 10; xterm -hold -e "cd catkin_ws; rosrun teleop_twist_joy joy_teleop.py"
# sleep 1, xterm -hold -e "cd ; realsense-viewer"
# roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

# roslaunch rtabmap_ros rtabmap.launch \
#     depth_topic:=/camera/aligned_depth_to_color/image_raw \
#     rgb_topic:=/camera/color/image_raw \
#     camera_info_topic:=/camera/color/camera_info \
#     approx_sync:=false \
#     rviz:=true \
#     rtabmapviz:=false \
#     localization:=true