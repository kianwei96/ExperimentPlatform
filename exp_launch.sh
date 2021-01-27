#!/bin/bash

xterm -iconic -hold -e "python postercombiner.py" &
sleep 3; xterm -iconic -hold -e "python gui.py" &
# sleep 1; xterm -iconic -hold -e "python controller_joystick.py" &
sleep 1; xterm -iconic -hold -e "./tcp-server 8080 8081"
