#!/bin/bash

xterm -iconic -hold -e "roscore" &
sleep 1; xterm -iconic -hold -e "cd ~/catkin_ws; roslaunch robot.launch" & #RosAria
sleep 1; xterm -iconic -hold -e "cd ~/catkin_ws; roslaunch hokuyo.launch" &
sleep 2; xterm -iconic -hold -e "cd ~/catkin_ws; rosrun robot_setup_tf tf_broadcaster" &
sleep 2; xterm -iconic -hold -e "cd ~/catkin_ws; rosrun rviz rviz" &
sleep 2; xterm -iconic -hold -e "cd ~/catkin_ws; rosrun map_server map_server ~/ExperimentPlatform/maptest1.yaml" &
sleep 2; xterm -iconic -hold -e "cd ~/catkin_ws; roslaunch AMCL.launch" &
sleep 2; xterm -iconic -hold -e "cd ~/catkin_ws; rostopic echo /amcl_pose"&
sleep 2; xterm -iconic -hold -e "cd ~/catkin_ws; rosrun joy joy_node _autorepeat_rate:=10"&
sleep 2; xterm -iconic -hold -e "cd ~/catkin_ws; rostopic echo joy"
#sleep 10; xterm -hold -e "cd catkin_ws; rosrun teleop_twist_joy joy_teleop.py"
