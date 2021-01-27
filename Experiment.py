from __future__ import with_statement

import requests
import curses
from psychopy import core
import sys
import select
import numpy as np
import csv
import random
import numpy.matlib
from std_msgs.msg import Int16
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
import pickle
import shutil
import subprocess
import math

with open('eparams.pkl','rb') as handle:
	exp_info = pickle.load(handle)

# terminal display

import os

date_folder = exp_info["sessionDate"]
shutil.copyfile('eparams.pkl', date_folder + '/eparams.pkl')

with open(os.devnull,'w') as fp:
	subprocess.Popen(['cd ~/ExperimentPlatform/' + str(date_folder) + '&& rosbag record -o posebag /amcl_pose /trigger_msgs __name:=my_bag'],shell=True,stdout=fp)
	subprocess.Popen(['cd ~/ExperimentPlatform/' + str(date_folder) + '&& rosbag record -o triggerbag /trigger_msgs __name:=my_bag2'],shell=True,stdout=fp)

resolution = 0.025000 # meters per cell

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
curses.curs_set(0)
stdscr.nodelay(1)

phase = "Pre Trial Interval"

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

# to pull from gui.py / map parameters

move_duration = 45
move_duration = exp_info["maze_trialDuration"]

cue_duration = 2
cue_duration = exp_info["maze_timeToStart"]

penalty_duration = 2
penalty_duration = exp_info["maze_penaltyDuration"]

reward_duration = 2
reward_duration = exp_info["maze_rewardDuration"]

inter_trial_interval = [5, 5] # 2 way inclusive
inter_trial_interval = [exp_info["maze_ITI_Min"], exp_info["maze_ITI_Max"]]

num_trials = 15
num_trials = exp_info["maze_numberOfTrials"]

reward_map = exp_info["maze_rewardMap"]

with open("RewardData/" + reward_map, 'r') as f:
	reader = csv.reader(f)
	poster_locations = list(reader)
poster_locations = poster_locations[1:]	
poster_count = len(poster_locations)
print(poster_count)

angle_tol = 90 # in degrees
angle_tol = exp_info["maze_angleTolerance"]

pos_tol = 20 # in cm
pos_tol = exp_info["maze_positionTolerance"]

dest_duration = 5
dest_duration = exp_info["maze_destinationDuration"]


# generating balanced, random reward sequence

reward_sequence = np.arange(poster_count)
reward_sequence = np.squeeze(np.transpose(np.matlib.repmat(reward_sequence, 1, 5)))

while True:
	np.random.shuffle(reward_sequence)
	evaluation = np.diff(np.transpose(reward_sequence))

	evaluation = evaluation == 0
	if not evaluation.any():
		break	

full_targets = reward_sequence

while len(full_targets) < num_trials:
	reward_sequence = np.arange(poster_count)
	reward_sequence = np.squeeze(np.transpose(np.matlib.repmat(reward_sequence, 1, 5)))
	while True:
    		np.random.shuffle(reward_sequence)
		evaluation = np.diff(np.transpose(reward_sequence)) == 0
		if not evaluation.any():
			break
	if full_targets[-1] == reward_sequence[0]:
		reward_sequence = reward_sequence + 1
		reward_sequence[reward_sequence == poster_count] = 0
	full_targets = np.hstack((full_targets, reward_sequence))

full_targets = np.transpose(full_targets)
targets = full_targets.tolist()

# setting up communication methods

target_location = [0, 0, 0]

'''
Check if alpha is between theta and beta (all in degrees)
'''
def is_angle_between(alpha, theta, beta):
    while( math.fabs(beta - alpha) > 180 ):
        if(beta > alpha):
            alpha += 360
        else:
            beta += 360


    # Here I replace alpha with beta if alpha is bigger to keep things consistent
    # You can choose the bigger angle however you please
    if(alpha > beta):
        phi = alpha
        alpha = beta
        beta = phi

    threeSixtyMultiple = (beta - theta)//360;
    theta += 360*threeSixtyMultiple

    return (alpha < theta) and (theta < beta)

def reached_target_location(data):
    (roll, pitch, angle) = euler_from_quaternion([data.pose.pose.orientation.x,
                                                  data.pose.pose.orientation.y, 
                                                  data.pose.pose.orientation.z, 
                                                  data.pose.pose.orientation.w])
    angle = math.degrees(angle)
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    has_reached_position =  (target_location[0] - x)**2 + \
                            (target_location[1] - y)**2 < (pos_tol/100)**2 # to meters
    convert_angle = lambda angle: angle - angle//(360)*(360)
    target_a = math.degrees(math.atan2((target_location[1]-y),(target_location[0]-x)))
    has_faced_target = is_angle_between(convert_angle(angle - angle_tol),
    									convert_angle(math.degrees(target_location[2]))+180,
    									convert_angle(angle + angle_tol))
    return has_reached_position and has_faced_target

def positionParser(data):
	
	global phase

	if phase == "Movement":
		pass
	else:
		return

	global master_timer
	global zone_entered
	dts = master_timer.getTime()

	global target_location # [x, y]
	global pos_saver # position for entire trial

	pos_saver.append([dts, data.pose.pose.position.x, data.pose.pose.position.y, sum([i**2 for i in data.pose.covariance])])

	if reached_target_location(data):		
		if zone_entered == -1:
			zone_entered = master_timer.getTime()
	else:
		zone_entered = -1


publisher = rospy.Publisher('trigger_msgs', Int16, queue_size=2)
marker = rospy.Publisher('current_poster', PointStamped, queue_size=2)
rospy.init_node('triggers', anonymous=True)
rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, positionParser)

def _sendHttpMsg(id):
	param = {'imageId': id}
	addr = 'http://localhost:8081/updateImage'
	r = requests.get(addr, params = param)

try:

	# initialization
	stdscr.refresh()
	stdscr.addstr(0, 0, "Poster sequence generated ... ")
	master_log = []
	master_log.append(["Marker", "Master Time"])
	with open(date_folder + "/sessionTriggers.csv", 'w') as storage:
		wr = csv.writer(storage, dialect='excel')
		wr.writerows(master_log)
		storage.close()
	master_log = []
	attempts = 0 # inclusive of failed attempts
	curr_trial = 0 # only tracks successful attempts
	curr_target = 0 # follows curr_trial, actually kind of redundant
	inter_trial_duration = np.random.randint(inter_trial_interval[0], inter_trial_interval[1]+1)
	master_timer = core.MonotonicClock() # reference clock, csv file will report with this time
	mini_timer = core.MonotonicClock() # to be used every event
	status = "Normal"
	phase = "Pre Trial Interval"
	encoding_dict = {"Cue Up":10, "Movement":20, "Reward":30, "Penalty":40}
	pt = PointStamped()
	pt.header.stamp = rospy.Time.now()
	pt.header.frame_id = '/map'
	pt.point.x = float(poster_locations[targets[curr_target]][0])
	pt.point.y = float(poster_locations[targets[curr_target]][1])
	pt.point.z = 0
	marker.publish(pt)
	target_location = [pt.point.x, pt.point.y, float(poster_locations[targets[curr_target]][2])]
	pos_saver = []
	zone_entered = -1

	while True:

		# Phase changes

		if phase == "Cue Up":
			if mini_timer.getTime() > cue_duration:
				phase = "Movement"
				zone_entered = -1
				message = Int16()
				message.data = encoding_dict[phase]+targets[curr_target]+1
				publisher.publish(message)
				master_log.append([encoding_dict[phase]+targets[curr_target]+1, master_timer.getTime()])
				mini_timer = core.MonotonicClock()

		if phase == "Movement":
			if mini_timer.getTime() > move_duration:
				phase = "Penalty"
				message = Int16()
				message.data = encoding_dict[phase]+targets[curr_target]+1
				publisher.publish(message)
				master_log.append([encoding_dict[phase]+targets[curr_target]+1, master_timer.getTime()])
				mini_timer = core.MonotonicClock()

		if phase == "Penalty":
			if mini_timer.getTime() > penalty_duration:
				phase = "Pre Trial Interval"
				inter_trial_duration = np.random.randint(inter_trial_interval[0], inter_trial_interval[1]+1)
				attempts = attempts + 1
				with open(date_folder + "/platformPositions.csv",'a') as storage:
					wr = csv.writer(storage, dialect='excel')
					wr.writerows(pos_saver)
					storage.close()
				pos_saver = []
				with open(date_folder + "/sessionTriggers.csv", 'a') as storage:
					wr = csv.writer(storage, dialect='excel')
					wr.writerows(master_log)
					storage.close()
				master_log = []
				pt = PointStamped()
				pt.header.stamp = rospy.Time.now()
				pt.header.frame_id = '/map'
				pt.point.x = float(poster_locations[targets[curr_target]][0])
				pt.point.y = float(poster_locations[targets[curr_target]][1])
				pt.point.z = 0
				marker.publish(pt)
				target_location = [pt.point.x, pt.point.y, float(poster_locations[targets[curr_target]][2])]
				mini_timer = core.MonotonicClock()
				if status == "Pausing":
					status = "Paused"
					phase = "Halted"	

		if phase == "Reward":
			if mini_timer.getTime() > reward_duration:
				if curr_trial == num_trials - 1:
					phase = "End of Session"
					with open(date_folder + "/platformPositions.csv",'a') as storage:
						wr = csv.writer(storage, dialect='excel')
						wr.writerows(pos_saver)
						storage.close()
					pos_saver = []
					with open(date_folder + "/sessionTriggers.csv", 'a') as storage:
						wr = csv.writer(storage, dialect='excel')
						wr.writerows(master_log)
						storage.close()
					master_log = []
					mini_timer = core.MonotonicClock()
				else:
					phase = "Pre Trial Interval"
					inter_trial_duration = np.random.randint(inter_trial_interval[0], inter_trial_interval[1]+1)
					curr_target = curr_target + 1
					curr_trial = curr_trial + 1
					attempts = attempts + 1
					with open(date_folder + "/platformPositions.csv",'a') as storage:
						wr = csv.writer(storage, dialect='excel')
						wr.writerows(pos_saver)
						storage.close()
					pos_saver = []
					with open(date_folder + "/sessionTriggers.csv", 'a') as storage:
						wr = csv.writer(storage, dialect='excel')
						wr.writerows(master_log)
						storage.close()
					master_log = []
					pt = PointStamped()
					pt.header.stamp = rospy.Time.now()
					pt.header.frame_id = '/map'
					pt.point.x = float(poster_locations[targets[curr_target]][0])
					pt.point.y = float(poster_locations[targets[curr_target]][1])
					pt.point.z = 0
					marker.publish(pt)
					target_location = [pt.point.x, pt.point.y, float(poster_locations[targets[curr_target]][2])]
					mini_timer = core.MonotonicClock()
					if status == "Pausing":
						status = "Paused"
						phase = "Halted"

		if phase == "Pre Trial Interval":
			if master_timer < 2.5:
				pt = PointStamped()
				pt.header.stamp = rospy.Time.now()
				pt.header.frame_id = '/map'
				pt.point.x = float(poster_locations[targets[curr_target]][0])
				pt.point.y = float(poster_locations[targets[curr_target]][1])
				pt.point.z = 0
				marker.publish(pt)
				target_location = [pt.point.x, pt.point.y, float(poster_locations[targets[curr_target]][2])]
			if mini_timer.getTime() > inter_trial_duration:
				phase = "Cue Up"
				message = Int16()
				message.data = encoding_dict[phase]+targets[curr_target]+1
				publisher.publish(message)
				_sendHttpMsg(targets[curr_target])
				master_log.append([encoding_dict[phase]+targets[curr_target]+1, master_timer.getTime()])
				mini_timer = core.MonotonicClock()
			
		########## control mechanisms ##########

		input = stdscr.getch()

		if input == 112: # p for pause
			if status == "Pausing":
				status = "Normal"
			elif status == "Normal":
				status = "Pausing"

		elif input == 114: # r for resume
			if phase == "Halted":
				phase = "Pre Trial Interval"
				status = "Normal"
				mini_timer = core.MonotonicClock()

		elif input == 116: # t for terminate, phase 1
			if phase == "Halted" or phase == "End of Session":
				phase = "Terminate?"

		elif input == 121 and phase == "Terminate?": # y for confirm terminate
			break

		elif input == 110 and phase == "Terminate?": # n to reject terminate
			phase = "Halted"				

		# elif input == 32 and phase == "Movement": # space for dummy hit target
		if zone_entered > 0 and master_timer.getTime() - zone_entered > dest_duration:		
			phase = "Reward"
			zone_entered = -1
			message = Int16()
			message.data = encoding_dict[phase]+targets[curr_target]+1
			publisher.publish(message)
			master_log.append([encoding_dict[phase]+targets[curr_target]+1, master_timer.getTime()])
			mini_timer = core.MonotonicClock()			

		########## display ##########

		stdscr.addstr(0, 0, "Trial Number   : {}".format(str(curr_trial+1).ljust(40, ' ')))
		stdscr.addstr(1, 0, "Attempt Number : {}".format(str(attempts+1).ljust(40, ' ')))
		stdscr.addstr(2, 0, "Target Poster  : {}".format(str(targets[curr_target]+1).ljust(40, ' ')))

		stdscr.addstr(4, 0, "Stage      : {}".format(phase).ljust(40, ' '))
		if phase == "Terminate?" or phase == "Halted" or phase == "End of Session":
			stdscr.addstr(5, 0, "Stage Time : -           ")
		else:
			stdscr.addstr(5, 0, "Stage Time : {}".format(str(round(mini_timer.getTime(),1)).ljust(40, ' ')))	
		stdscr.addstr(6, 0, "Status     : {}".format(status).ljust(40, ' '))

		stdscr.refresh()

finally:

	subprocess.call(['xterm','-e','cd ~/ExperimentPlatform && rosnode kill my_bag'])
	subprocess.call(['xterm','-e','cd ~/ExperimentPlatform && rosnode kill my_bag2'])
	with open(os.devnull,'w') as fp:
		subprocess.Popen(['cd ~/ExperimentPlatform/' + str(date_folder) + '&& rostopic echo -b posebag*.bag -p /amcl_pose > rospose.csv'],shell=True,stdout=fp)
	with open(os.devnull,'w') as fp:	
		subprocess.Popen(['cd ~/ExperimentPlatform/' + str(date_folder) + '&& rostopic echo -b triggerbag*.bag -p /trigger_msgs > rostrig.csv'],shell=True,stdout=fp)
	eos = Int16()
	eos.data = 55
	publisher.publish(eos)
	curses.echo()
	curses.nocbreak()
	curses.endwin()
	

