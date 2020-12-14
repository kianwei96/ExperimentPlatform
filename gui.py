import sys
import os
#import reward_modifier

from PyQt5 import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtCore import QDir
from PyQt5.uic import loadUiType
from PyQt5.uic import loadUi
# import main 
import subprocess
from psychopy import core
import pickle
import shutil

import numpy as np
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import String
from math import cos, sin, atan, asin, pi

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
#import StartExperiment_amcl_v5_GUI 
import utils
import json
import cv2
import time
import datetime


Ui_MainWindow, QMainWindow = loadUiType("gui.ui")


class Main(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(Main, self).__init__()


        self.setupUi(self)
        self.start_button.clicked.connect(self.start)
        #self.maze_startButton.clicked.connect(self.maze_start)
        self.load_button.clicked.connect(self.load)
        
        #RewardLocationCSV = '/home/sinapse/Desktop/MonkeyGUI-master/RewardData/rewardlocations.csv'
        #Rewardlocation = [[0, 0], [0, 0]] #pd.read_csv(RewardLocationCSV, index_col=0)
        #RewardLocation = len(Rewardlocation)
        #print 'Number of saved reward locations loaded : ', RewardLocation

        self.subject.setText("r")
        self.session.setText("1")
        self.juicer_height.setText("100")
        self.juicer_height.textChanged.connect(self.calculate_flowrate)
        self.calculate_flowrate()
        self.fixation_criterion.setText("1.0")
        self.fixation_grace.setText("0.3")
        self.eccentricity.setText("5.0")
        self.autocycle.setChecked(False)
        self.intertrial_interval.setText("1.0")

        self.trial_timeout.setText("100.0")
        self.fixation_size.setText("1.0")
        self.fixation_window.setText("1.0")
        self.fixation_buffer.setText("0.0")
        self.eyespot.setChecked(False)
        self.use_dummy.setChecked(True)
        self.calibrate.setChecked(True)
        self.play_sound.setChecked(True)
        self.block_report.setChecked(False)

        # monitor related stuff
        self.screen_distance.setText("57")
        self.screen_size.setText("22")
        self.screen_height.setText("1050")
        self.screen_width.setText("1680")

        self.mask_params = {}
        self.fixation_mask.addItem("square")
        self.fixation_mask.addItem("gauss")
        self.fixation_mask.addItem("circle")
        self.fixation_mask.addItem("bullseye")
        self.fixation_mask.activated.connect(self.setMaskParams)
        self._combination_chooser = None

        # calibration stuff
        self.calibration_reward_duration.setText("0.5")
        self.calibration_target_size.setText("1.0")
        self.calibration_type.addItem("3 points")
        self.calibration_type.addItem("5 points")
        self.calibration_type.addItem("9 points")
        self.calibration_type.addItem("13 points")
        self.calibration_target_color.addItem("white")
        self.calibration_target_color.addItem("yellow")
        self.calibration_target_color.addItem("blue")
        self.calibration_stimulus.addItem("Image...")
        self.calibration_stimulus.addItem("Gabor patch")
        self.calibration_stimulus.addItem("Circle")
        self.calibration_stimulus.activated.connect(self.set_calibration_image)
        self.manual_calibration.setChecked(False)
       
	self.choose_rewards.addItem("")
	for file in os.listdir("RewardData"):
		if file.endswith(".csv"):
			self.choose_rewards.addItem(file)
	
	#self.choose_rewards.activated.connect(self.show_rewards)	
	#self.mod_rewards.clicked.connect(self.modify_rewards)
	
	self.choose_rewards.activated.connect(self.set_rewards)

        self.maze_numberOfTrials.setText("30")
        self.maze_angleTolerance.setText("90")
        self.maze_positionTolerance.setText("20") #in CM
        self.maze_destinationDuration.setText("2")
	self.maze_rewardDuration.setText("10")
	self.maze_penaltyDuration.setText("5")
        self.maze_trialDuration.setText("120") #in Seconds
        self.maze_timeToStart.setText("5") #in Seconds
        self.maze_ITI_Min.setText("1")    #in Seconds
        self.maze_ITI_Max.setText("3")   #in Seconds

        # platform stuff
        self.platform_clear_dist.setText("1.6") #need convert to metres default 1.2 [1.6 = 80cm slow]
        self.platform_stop_dist.setText("1.0")  #need convert to metres default 0.7 [1.0 = 60cm stop]
        self.platform_slowDownSpeed.setText("0.1") 
        self.platform_normalSpeed.setText("0.2")

        # saveas
        self.saveas_button.clicked.connect(self.saveas)

        # reward stuff
        self.reward_duration.setText("0.5")
        self.manual_reward_duration.setText("0.5")
        self.base_reward_duration.setText("0.5")

        # results
        self.results = {}
        self.combos = []

    def set_rewards(self):
	if self.choose_rewards.currentText() != "":
		publisher = rospy.Publisher('file_name', String, queue_size=1)
		rospy.init_node('gui_selection', anonymous=True)
		publisher.publish(self.choose_rewards.currentText())
	#reward_modifier.update_main(self.choose_rewards.currentText())

    def scale_reward_changed(self, int):
        if self.scale_reward.isChecked():
            self.base_reward_duration.setEnabled(True)
        else:
            self.base_reward_duration.setEnabled(False)

    def calculate_flowrate(self):
        juicer_height = self.juicer_height.text()
        if juicer_height:
            y = float(juicer_height)
            self.droprate.setText("%.3f" % (utils.calc_flowrate(y), ))


    def setMaskParams(self):
        if self.fixation_mask.currentText() == "gauss":
            text, ok = QtGui.QInputDialog.getText(self, "Mask parameter", "Standard deviation: ")
            self.mask_params = {"sd": float(text)}
        else:
            self.mask_params = {}

    def choose_combinations(self):
        if self._combination_chooser is None:
            num_targets = int(self.num_targets.text())
            self._combination_chooser = QtGui.QDialog(self)
            self._combination_chooser.resize(200, 200)
            main_layout = QtGui.QVBoxLayout()
            _textfield = QtGui.QListWidget()
            _line_edit = QtGui.QLineEdit()
            # populate the box
            for cc in self.combos:
                _textfield.addItem(cc)

            def _copy_text():
                _ss = str(_line_edit.text())
                if len(_ss) > num_targets:
                    # this is not an allowed combination
                    _line_edit.setText("")
                else:
                    _items = _textfield.findItems(_line_edit.text(), QtCore.Qt.MatchExactly)
                    if len(_items) == 0:
                        _textfield.addItem(_line_edit.text())
                    else:
                        # update the item at _idx
                        _items[0].setText(_line_edit.text())
                    if _ss not in self.combos:
                        self.combos.append(_ss)
                    _line_edit.setText("")


            def _retrieve_text(item):
                _line_edit.setText(item.text())

            def _delete_item():
                listItems = _textfield.selectedItems()
                if not listItems: return
                for item in listItems:
                    _textfield.takeItem(_textfield.row(item))
                    self.combos.remove(str(item.text()))

            _textfield.connect(QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Delete), _textfield),
                             QtCore.SIGNAL('activated()'), _delete_item)
            _textfield.connect(QtGui.QShortcut(QtGui.QKeySequence(QtCore.Qt.Key_Backspace), _textfield),
                               QtCore.SIGNAL('activated()'), _delete_item)
            _textfield.itemActivated.connect(_retrieve_text)

            _line_edit.returnPressed.connect(_copy_text)
            main_layout.addWidget(_line_edit)
            main_layout.addWidget(_textfield)
            self._combination_chooser.setLayout(main_layout)

        self._combination_chooser.exec_()

    def set_calibration_image(self):
        if self.calibration_stimulus.currentText() == "Image...":
            filters = "Image and movie files (*.jpg *.tiff *.png *.mp4)"
            
            filename = QtGui.QFileDialog.getOpenFileName(self, "Set calibration image", os.getcwd(),
                                                         filters)
            if filename:
                idx = self.calibration_stimulus.findText(filename, QtCore.Qt.MatchFixedString)
                if idx < 0:  # not found, so add it
                    self.calibration_stimulus.addItem(filename)
                    idx = self.calibration_stimulus.findText(filename, QtCore.Qt.MatchFixedString)
                self.calibration_stimulus.setCurrentIndex(idx)


    def load(self):
        filters = "Settings files (*.txt)"
        # TODO: Why is this not resolving symoblic links?
        #filename = QtGui.QFileDialog.getOpenFileName(self, "Load settings", os.getcwd(),filters)
        filename, _filer = QtWidgets.QFileDialog.getOpenFileName(self, "Load settings", os.getcwd(),filters)

        if filename:
            exp_info = json.load(open(str(filename), "r"))
            self.subject.setText(exp_info.get("subject", "r"))
            self.juicer_height.setText(str(exp_info.get("juicer_height", 100.0)))
            self.calculate_flowrate()
            self.exp_plot.setChecked(exp_info.get("exp_plot", False))
            self.use_dummy.setChecked(exp_info.get("use_dummy", True))
            self.eyespot.setChecked(exp_info.get("draw_eyespot", True))
            self.screen_width.setText(str(exp_info.get("screen_width", 1680)))
            self.screen_height.setText(str(exp_info.get("screen_height", 1080)))
            self.screen_size.setText(str(exp_info.get("screen_size", 25)))
            self.screen_distance.setText(str(exp_info.get("screen_distance", 57)))
            self.fixation_criterion.setText(str(exp_info.get("fixation_criterion", 1.0)))
            self.fixation_grace.setText(str(exp_info.get("fixation_grace", 0.3)))
            self.fixation_buffer.setText(str(exp_info.get("fixation_buffer", "0.0")))
            self.eccentricity.setText(str(exp_info.get("eccentricity", 5.0)))
            self.autocycle.setChecked(exp_info.get("autocycle", False))
            self.intertrial_interval.setText(str(exp_info.get("intertrial_duration", 1.0)))
            self.reward_duration.setText(str(exp_info.get("reward_duration", 0.5)))
            self.scale_reward.setChecked(exp_info.get("scale_reward", False))
            self.base_reward_duration.setEnabled(exp_info.get("scale_reward", False))
            self.base_reward_duration.setText(str(exp_info.get("base_reward_duration", 0.5)))
            make_response = exp_info.get("make_response", False)
            if make_response:
                idx = self.reward_condition.findText("Saccade to target", QtCore.Qt.MatchFixedString)
            else:
                idx = self.reward_condition.findText("Maintain fixation", QtCore.Qt.MatchFixedString)
            self.reward_condition.setCurrentIndex(idx)

            self.manual_reward_duration.setText(str(exp_info.get("manual_reward_duration", 0.5)))
            self.manual_reward_duration_2.setText(str(exp_info.get("manual_reward_duration_2", 0.5)))
            self.serial_path.setText(str(exp_info.get("serial_port", "")))
            self.trial_timeout.setText(str(exp_info.get("trial_timeout", 100.0)))
            self.fixation_size.setText(str(exp_info.get("fixation_size", 1.0)))
            self.fixation_window.setText(str(exp_info.get("fixation_window", 1.0)))

            self.play_sound.setChecked(bool(exp_info.get("pay_sound", True)))
            self.block_report.setChecked(bool(exp_info.get("block_report", False)))

            # combobox
            fixmask = exp_info.get("fixation_mask", "circle")
            idx = self.fixation_mask.findText(fixmask, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.fixation_mask.setCurrentIndex(idx)
            rcolor = str(exp_info.get("reward_color", None))
            idx = self.reward_color.findText(rcolor, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.reward_color.setCurrentIndex(idx)
            fcolor = str(exp_info.get("failure_color", None))
            idx = self.failure_color.findText(fcolor, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.failure_color.setCurrentIndex(idx)

            #  calibration
            self.calibration_reward_duration.setText(str(exp_info.get("calibration_reward_duration", 0.5)))
            self.calibration_target_size.setText(str(exp_info.get("calibration_target_size", 1.0)))
            calib_type = exp_info.get("calibration_type", "9 points")
            idx = self.calibration_type.findText(calib_type, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.calibration_type.setCurrentIndex(idx)
            calib_color = exp_info.get("calibration_target_color", "white")
            idx = self.calibration_target_color.findText(calib_color, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.calibration_target_color.setCurrentIndex(idx)

            calibration_stim = exp_info.get("calibration_stimulus", "Circle")
            idx = self.calibration_stimulus.findText(calibration_stim)
            if idx >= 0:
                self.calibration_stimulus.setCurrentIndex(idx)
            else:
                self.calibration_stimulus.addItem(calibration_stim)
            self.manual_calibration.setChecked(exp_info.get("manual_calibration", False))

            # target
            _target_onset_ref = exp_info.get("target_onset_ref", "Fixation")
            idx = self.target_onset_ref.findText(_target_onset_ref, QtCore.Qt.MatchFixedString)
            if idx >= 0:
                self.target_onset_ref.setCurrentIndex(idx)
            self.num_targets.setText(str(exp_info.get("num_targets", 0)))
            self.target_onset.setText(str(exp_info.get("target_onset", 0.0)))
            self.target_size.setText(str(exp_info.get("target_size", 2.0)))
            self.target_duration.setText(str(exp_info.get("target_duration", 1.0)))
            self.inter_target_interval.setText(str(exp_info.get("inter_target_interval", 0.0)))
            target_locations = exp_info.get("target_locations", [])
            self.target_locations.setPlainText("\n".join([str(t).strip("[]") for t in target_locations]))
            self.target_anchors.setChecked(bool(exp_info.get("show_anchors", False)))
            self.repeat_locations.setChecked(bool(exp_info.get("repeat_locations", False)))

            if "response_cue_onset" in exp_info.keys():
                self.response_cue_onset_min.setText(str(exp_info.get("response_cue_onset", 0.0)))
                self.response_cue_onset_max.setText(str(exp_info.get("response_cue_onset", 0.0)))
            else:
                self.response_cue_onset_min.setText(str(exp_info.get("response_cue_onset_min", 0.0)))
                self.response_cue_onset_max.setText(str(exp_info.get("response_cue_onset_max", 0.0)))
            response_cue_ref = exp_info.get("response_cue_ref", "Target offset")
            idx = self.response_cue_ref.findText(response_cue_ref)
            if idx >= 0:
                self.response_cue_ref.setCurrentIndex(idx)

            self.max_rtime.setText(str(exp_info.get("max_rtime", 0.0)))
            self.max_saccade_time.setText(str(exp_info.get("max_saccade_time", 0.0)))
            self.target_fix_time.setText(str(exp_info.get("target_fix_time", 0.0)))
            self.target_window_size.setText(str(exp_info.get("target_window_size", self.target_size.text())))
            self.repeat_failed_targets.setText(str(exp_info.get("repeat_failed_targets", 0)))
            self.combos = exp_info.get("allowed_combos", [])

            #Maze
            #self.maze_rewardLocation.setText(str(exp_info.get("maze_rewardlocation", 3.0)))
	    self.maze_numberOfTrials.setText(str(exp_info.get("maze_numberOfTrials", 3.0)))
            self.maze_angleTolerance.setText(str(exp_info.get("maze_angleTolerance", 90)))
            self.maze_positionTolerance.setText(str(exp_info.get("maze_positionTolerance", 15)))
            self.maze_destinationDuration.setText(str(exp_info.get("maze_destinationDuration", 2.0)))
	    self.maze_rewardDuration.setText(str(exp_info.get("maze_rewardDuration", 10.0)))
	    self.maze_penaltyDuration.setText(str(exp_info.get("maze_penaltyDuration", 5.0)))
            self.maze_trialDuration.setText(str(exp_info.get("maze_trialDuration", 60.0)))
            self.maze_timeToStart.setText(str(exp_info.get("maze_timeToStart", 5.0)))
            self.maze_ITI_Min.setText(str(exp_info.get("maze_ITI_Min", 1.0)))
            self.maze_ITI_Max.setText(str(exp_info.get("maze_ITI_Max", 5.0)))

            #Platform
            self.platform_clear_dist.setText(str(exp_info.get("platform_clear_dist", 1.2)))
            self.platform_stop_dist.setText(str(exp_info.get("platform_stop_dist", 0.7)))
            self.platform_normalSpeed.setText(str(exp_info.get("platform_normalSpeed", 0.2)))
            self.platform_slowDownSpeed.setText(str(exp_info.get("platform_slowDownSpeed", 0.1)))


    def saveas(self):
        #filename = QtGui.QFileDialog.getSaveFileName(self, "Save settings", os.getcwd())
        filename = QtWidgets.QFileDialog.getSaveFileName(self, "Save settings", os.getcwd())
        
        if filename:
            exp_info = self.get_settings(0)
            print(str(filename[0]))
	    json.dump(exp_info, open(str(filename[0])+'.txt',"w"))
            #json.dump(exp_info, open(filename, "w"))

    def show_target_preview(self):
        exp_info = self.get_settings()
        main.target_preview(exp_info)

    def get_settings(self, final):
        
        today = datetime.date.today()
        d1 = today.strftime("%Y%m%d")

	if final == 1:
	    if not os.path.exists(d1):
	        os.makedirs(d1)
	    else:
	        shutil.rmtree(d1)
	        os.makedirs(d1)
	
	subject = str(self.subject.text())
        session = int(self.session.text())
        droprate = float(self.droprate.text())
        screen_width = float(self.screen_width.text())
        screen_height = float(self.screen_height.text())
        screen_distance = float(self.screen_distance.text())
        screen_size = float(self.screen_size.text())
        serialpath = str(self.serial_path.text())
        eccentricity = float(self.eccentricity.text())
        autocycle = self.autocycle.isChecked()
        fixcrit = float(self.fixation_criterion.text())
        fixation_grace = float(self.fixation_grace.text())
        fixation_buffer = float(self.fixation_buffer.text())
        iti = float(self.intertrial_interval.text())
        reward_duration = float(self.reward_duration.text())
        base_reward_duration = float(self.base_reward_duration.text())
        manual_reward_duration = float(self.manual_reward_duration.text())
        trial_timeout = float(self.trial_timeout.text())
        fixsize = float(self.fixation_size.text())
        fix_window = float(self.fixation_window.text())
        use_dummy = self.use_dummy.isChecked()
        play_sound = self.play_sound.isChecked()
        block_report = self.block_report.isChecked()
        use_eyespot = self.eyespot.isChecked()
        exp_plot = self.exp_plot.isChecked()
        fixation_mask = str(self.fixation_mask.currentText())
        calibrate = self.calibrate.isChecked()
        # calibration stuff
        calibration_reward_duration = float(self.calibration_reward_duration.text())
        calibration_target_size = float(self.calibration_target_size.text())
        calibration_target_color = str(self.calibration_target_color.currentText())
        calibration_type = str(self.calibration_type.currentText())
        calibration_stimulus = str(self.calibration_stimulus.currentText())
        manual_calibration = self.manual_calibration.isChecked()

        # Maze stuff
        #maze_rewardLocation = float(self.maze_rewardLocation.text())
        maze_numberOfTrials = float(self.maze_numberOfTrials.text())
        maze_angleTolerance = float(self.maze_angleTolerance.text())
        maze_positionTolerance = float(self.maze_positionTolerance.text())
        maze_destinationDuration = float(self.maze_destinationDuration.text())
        maze_rewardDuration = float(self.maze_rewardDuration.text())
	maze_penaltyDuration = float(self.maze_penaltyDuration.text())
	maze_trialDuration = float(self.maze_trialDuration.text())
        maze_timeToStart = float(self.maze_timeToStart.text())
        maze_ITI_Min = float(self.maze_ITI_Min.text())
        maze_ITI_Max = float(self.maze_ITI_Max.text())
	maze_rewardMap = str(self.choose_rewards.currentText())

        # Platform stuff
        platform_clear_dist = float(self.platform_clear_dist.text())
        platform_stop_dist = float(self.platform_stop_dist.text())
        platform_normalSpeed = float(self.platform_normalSpeed.text())
        platform_slowDownSpeed = float(self.platform_slowDownSpeed.text())

        #combo selection
        allowed_combos = self.combos
        # increment the session counter for the next session
        self.session.setText(str(session+1))
        exp_info = {"sessionDate": d1,
		    "subject": subject,
                    "session": session,
                    "droprate": droprate,
                    "screen_width": screen_width,
                    "screen_height": screen_height,
                    "screen_size": screen_size,
                    "screen_distance": screen_distance,
                    "serial_port": serialpath,
                    "fixation_criterion": fixcrit,
                    "fixation_grace": fixation_grace,
                    "eccentricity": eccentricity,
                    "autocycle": autocycle,
                    "intertrial_duration": iti,
                    "reward_duration": reward_duration,
                    "base_reward_duration": base_reward_duration,
                    "manual_reward_duration": manual_reward_duration,
                    "trial_timeout": trial_timeout,
                    "fixation_size": fixsize,
                    "fixation_window": fix_window,
                    "fixation_buffer": fixation_buffer,
                    "fixation_mask": fixation_mask,
                    "mask_params": self.mask_params,
                    "draw_eyespot": use_eyespot,
                    "use_dummy": use_dummy,
                    "play_sound": play_sound,
                    "block_report": block_report,
                    "calibrate": calibrate,
                    "exp_plot": exp_plot,
                    "calibration_reward_duration": calibration_reward_duration,
                    "calibration_target_size": calibration_target_size,
                    "calibration_type": calibration_type,
                    "calibration_target_color": calibration_target_color,
                    "calibration_stimulus": calibration_stimulus,
                    "manual_calibration": manual_calibration,
                    "allowed_combos": allowed_combos,
		    "maze_rewardMap": maze_rewardMap,
                    #"maze_rewardLocation": maze_rewardLocation,
                    "maze_numberOfTrials": maze_numberOfTrials,
                    "maze_angleTolerance": maze_angleTolerance,
                    "maze_positionTolerance" : maze_positionTolerance,
                    "maze_destinationDuration": maze_destinationDuration,
		    "maze_rewardDuration": maze_rewardDuration,
		    "maze_penaltyDuration": maze_penaltyDuration,
                    "maze_trialDuration": maze_trialDuration,
                    "maze_timeToStart": maze_timeToStart,
                    "maze_ITI_Min": maze_ITI_Min,
                    "maze_ITI_Max": maze_ITI_Max,
                    "platform_stop_dist": platform_stop_dist,
                    "platform_clear_dist": platform_clear_dist,
                    "platform_normalSpeed": platform_normalSpeed,
                    "platform_slowDownSpeed": platform_slowDownSpeed}
        return exp_info


    def start(self):
        exp_info = self.get_settings(1)
        print "Starting experiment"
        #dump info
	with open('eparams.pkl','wb') as handle:
		pickle.dump(exp_info, handle, protocol=pickle.HIGHEST_PROTOCOL)
	#call exp
	subprocess.call(['xterm', '-e', 'cd ~/ExperimentPlatform && python Experiment.py'])
	

if __name__ == "__main__":
    import sys 

    app = QtWidgets.QApplication(sys.argv)
    myapp = Main()
    myapp.show()
    sys.exit(app.exec_())
