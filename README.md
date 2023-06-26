# ExperimentPlatform
Real-world maze navigation with tracking and experimental data collection.

Required scripts and files are mostly contained within this directory, which is to be located at `~/ExperimentPlatform`. Program expects ROS environment to be already set up at `~/catkin_ws`.

### Remote Control and Connectivity

Both platforms have been set up to automatically log in and connect to the Belkin router on startup, with the network name `powerbot`. The router reserves `192.168.0.2` for the older platform, and `192.168.0.3` for the newer platform. To control through VNC, use `vnc://192.168.0.3::5900` in TightVNC Viewer on Windows. Headless access can be made through `ssh sinapse@192.168.0.X` from a lab workstation.

### Re-mapping an environment

1. In a terminal window, navigate to `ExperimentPlatform` and run `mapping_launch.sh`
```
cd ~/ExperimentPlatform
bash mapping_launch.sh
```
2. Navigate the environment slowly with the main joystick, monitoring the mapping on Rviz
3. Once done, run the following in a new terminal, replacing <map_name> with a file name of your choice
```
rosrun map_server map_saver -f <map_name>
```
For example, to save in the `ExperimentPlatform` folder:
```
E.g. rosrun map_server map_saver -f  /home/sinapse/ExperimentPlatform/new_map
```
4. A .pgm and .yaml file will be generated, transfer them to the `ExperimentPlatform` folder if not already done so
5. Edit line 8 in `ros_launch.sh` to point to the new map OR rename new map to match the script

### Running the experiment

1. In a terminal window, navigate to `ExperimentPlatform` and run 'ros_launch.sh`, which sets up the required ROS services.
```
cd ~/ExperimentPlatform
bash ros_launch.sh
```
2. Start up the iOS app Runner on the iPhone. 
3. In another terminal window, navigate to `ExperimentPlatform` and run `exp_launch.sh` to run required python scripts that control experiment behavior.
```
cd ~/ExperimentPlatform
bash exp_launch.sh
```
4. In Rviz, get a good localization of the platform by using `P` to make a initial guess on position, and driving it around slightly
5. In the GUI window that pops up, Select the reward map under the `Reward Locations` drop-down list
6. To modify the reward map, use the `G` key in Rviz to redefine the 6 poster locations/directions (click and drag), with FIFO overriding
7. 7.	To create a new reward map, create a new csv file in the RewardData folder. In the GUI window, choose the new csv file from the drop-down list. Use the G key in Rviz to define the poster locations/direction (click and drag). The csv file will update the data accordingly.
8. Load or modify other experiment settings, and save the settings if you wish
9. Press `Start`

Rviz will now display current goal. In the new window that pops up, you can monitor the experiment progress and intervene as required. 
There are mainly 4 phases (Pre Trial Interval, Start Trial, Cue Offset, Reward/Timeout) that you will encounter during each trial. Only at Cue Offset the robot will not be programmically unclamped and it is a cue for you to move the robot.
To intervene, press `P` to toggle Pause behavior. Note that the experiment will only pause at the end of the current trial. Once end of trial is reached with the Pause option, use `R` to resume the experiment, or `T` followed by `Y` to terminate the entire experiment. At Paused and the first phase of Terminate (when `T` is first pressed), the robot will go into clamp mode if it was in a moving mode previously. This is to prevent any unwanted movement and can be used in emergency cases. Do NOT quit the experiment by any other method (eg. closing the window) as that will cause some ROS data to be recorded continuously. See below for remedy. 

If the program exits ungracefully, open a new window and run
```
rosnode kill my_bag
rosnode kill my_bag2
```

### End of Experiment

At the end of all the trials, terminate the experiment with `T` `Y`. You can press `Start` again in the GUI to re-run the whole experiment from Trial 1.

Experimental data will be stored in a folder named as the current date. A brief explanation of what each file contains:

* **eparams.pkl** - pickled python dictionary that stores parameters defined in the GUI
* **platformPositions.csv** - pose data timestamped with psychopy clock [time, dim1, dim2, sum(covariance)]
* **sessionTriggers.csv** - trigger values, timestamped with psychopy clock
* **rospose.csv** - raw pose data from ROS with ROS timing
* **posebag_???-??-??-??-??-??.bag** - rosbag data file, parent of rospose.csv
* **rostrig.csv** - trigger values, with ROS timings
* **pose_bag???-??-??-??-??-??.bag** - rosbag data file, parent of rostrig.csv

### Recording of full experiments
1. save bag file with 
```
rosbag record -a -o data.bag -x “/RosAria/*”
```
2. playback the rosbag file
```
ros set use_sim_time true && rosrun rviz rviz -d rviz_config/default.rviz
# in another terminal
rosbag play <bag_file>.bag
```
3. convert to csv file
```
rostopic echo <topic_name> -b <bag_file>.bag > <csv_file>.csv
```

### Eyelink
1. Eyelink Calibration is done via [this](https://github.com/grero/EyelinkCalibration)
```
# in `powerbot` terminal
cd ~/EyelinkCalibration/EyelinkCalibration && python main_gui.py
```
2. run calibrate_video.py after successful calibration for set 1 experiment
3. super-position results can be found [here](https://github.com/ndhuu/fyp_result/blob/main/eyelink.ipynb). This is slightly different from Matlab version (using 'v4' griddata, wherease python's griddata does not have 'v4', hence failed for extrapolation)
4. After calibration is done, you can physically place markers in md2 maze room to denote as the reward locations. You will then need to modify the reward locations in rviz to match the ones you placed. You can check with the researchers in md2 as to where the fixed reward locations will be at.
5. Ensure the eyelink checkbox is marked in the GUI to enable eyelink for the experiment
6. Follow the trial phases and rewards and look at the markers at each reward locations. Make sure your eye is being detected properly by the eyelink computer. It should be marked as green in the eyelink computer if it is detected correctly.
7. After the whole experiment is done, the robot will save the eyelink data file (.edf) in /data folder


### For Developers

Please refer to [Developer's Guide](../../wiki/Developer's-Guide)
 for breakdown of how the python scripts interact. Possible features and suggestions will also be listed there.

### For Localisation Testing

Please refer to `Localisation_README.md` for more information and guide

