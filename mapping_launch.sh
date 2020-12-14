#!/bin/bash

xterm -iconic -hold -e "roscore" &
sleep 1; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; roslaunch robot.launch" & #RosAria
sleep 1; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; roslaunch hokuyo.launch" &
sleep 2; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; rosrun robot_setup_tf tf_broadcaster" &
sleep 5; xterm -iconic -hold -e "cd ~/ExperimentPlatform/launch; roslaunch gmapping.launch"
