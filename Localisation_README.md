# Localisation Test

1. Create target.txt (if it is not already in /ExperimentPlatform folder). Format is 3 columns: Target angle, Target X, Target Y.
  
The number of rows should correspond with the number of checkpoints. You can either put the actual values of each checkpoints or simply just a fake number. One way to do it is to put the checkpoint number like 1,1,1 and 2,2,2 to denote checkpoint 1 and 2 respectively. This file is used for identification purposes, to identify which data points is in which checkpoint.

In a terminal window 1, navigate to `ExperimentPlatform` and run `ros_launch.sh`
```
cd ~/ExperimentPlatform
bash ros_launch.sh
```
In another terminal window 2, run the localisation script to start
```
python map_test.py
```
In order to record the data while navigating the robot, run the recording command in another terminal window 3
```
rosbag record -a -o data.bag -x "/RosAria/*"
```

The localisation test works in this way: first round is where you will compare the other rounds of results with. So the first round is very crucial.

Basically navigate the robot to the checkpoint markers set and click the target button on the joystick.

The terminal window 2 will prompt `Enter target index: `. Enter the checkpoint number and click `Enter`. Do note that checkpoint number starts from zero. After which, click the continue button on joystick. Move onto the next checkpoint marker and do the same till the end of experiment.

After experiment has completed, exit from terminal window 3 by keyboard interrupt (crtl-c). The rosbag file will be saved automatically.

Convert the rosbag file to csv format for processing
```
rostopic echo test_result -b data_<date-of-experiment>-<time-of-experiment>.bag > <csv file name>
```

