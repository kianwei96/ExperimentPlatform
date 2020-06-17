#!/bin/bash

xterm -iconic -hold -e "roscore" &
sleep 1; xterm -iconic -hold -e "cd ~/catkin_ws; roslaunch robot.launch" & #RosAria
sleep 1; xterm -iconic -hold -e "cd ~/catkin_ws; roslaunch hokuyo.launch" &
sleep 2; xterm -iconic -hold -e "cd ~/catkin_ws; rosrun robot_setup_tf tf_broadcaster" &
sleep 2; xterm -iconic -hold -e "cd ~/catkin_ws; rosrun rviz rviz" &
sleep 5; xterm -iconic -hold -e "cd ~/catkin_ws; roslaunch gmapping.launch"
