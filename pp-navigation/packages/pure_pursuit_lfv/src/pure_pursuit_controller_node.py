#!/usr/bin/env python

import math
import time
import numpy as np
import rospy
import cv2
from duckietown_msgs.msg import Twist2DStamped, LanePose, Segment, SegmentList, BoolStamped, WheelsCmdStamped, ObstacleProjectedDetection, ObstacleProjectedDetectionList
from pure_pursuit.utils import Gearbox, Buffer, drawImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import copy
import collections

"""
NOTE: In our experience, parameters can vary depending on the simulator and how well the duckiebot
is calibrated. Proper tuning of parameters makes a big difference in performance. Parameters can be
changed using "rosparam set [param_name] [param_value]" to make tuning more convenient. The most sensitive
parameters are the following: lane_width, v_min, v_max, dv_neg, dv_pos, w_gain_min, w_gain_max

Visit https://github.com/saryazdi/Duckietown-Object-Detection-LFV for more information.
"""

class pure_pursuit(object):
	def __init__(self):
		self.node_name = rospy.get_name()
		self.node_namespace = rospy.get_namespace()
		Parameters = collections.namedtuple('parameters', 'v_min, v_max, dv_neg, dv_pos, w_gain_min, w_gain_max, func, num_y_vup, num_r_vdown, right_wg_scale, obs_stop_thresh, lane_width, robot_width, white_dir_correction, yellow_dir_correction')
		self.loginfo('NAMESPACE: %s' % str(self.node_namespace))
		self.loginfo('####################')
		self.loginfo('### PURE_PURSUIT ###')
		self.loginfo('####################')
		self.loginfo('Using pure_pursuit')
		if self.node_namespace == '/default/':
			# Simulation parameters
			self.loginfo('***********************')
			self.loginfo('*** SIMULATION MODE ***')
			self.loginfo('***********************')
			self.loginfo('Using simulation parameters for pure pursuit')
			self.lookahead_distance = 0.25
			params = Parameters(v_min=0.3, v_max=0.7, dv_neg=-0.25, dv_pos=0.03, w_gain_min=0.62, w_gain_max=1.5, num_y_vup=1, num_r_vdown=1, right_wg_scale=1.05, func='x2', obs_stop_thresh=0.3, lane_width=0.36, robot_width=0.12, white_dir_correction=0.5, yellow_dir_correction=0.25)
		else:
			# Hardware parameters
			self.loginfo('*********************')
			self.loginfo('*** HARDWARE MODE ***')
			self.loginfo('*********************')
			self.loginfo('Using hardware parameters for pure pursuit')
			self.lookahead_distance = 0.25
			params = Parameters(v_min=0.2, v_max=0.5, dv_neg=-0.25, dv_pos=0.03, w_gain_min=0.62, w_gain_max=2.6, num_y_vup=1, num_r_vdown=3, right_wg_scale=1.05, func='x2', obs_stop_thresh=0.3, lane_width=0.36, robot_width=0.12, white_dir_correction=0.5, yellow_dir_correction=0.5) # safe and steady

		# Ros parameters
		self.rosparamlist = ['verbose', 'vehicle_avoidance']
		self.verbose = rospy.get_param('~verbose', False)
		self.vehicle_avoidance = rospy.get_param('~vehicle_avoidance', True)
		for (key, value) in params._asdict().items():
			value = self.setupParam('~' + key, value)
			setattr(self, key, value)
		self.lane_width = self.setupParam('~lane_width', params.lane_width)
		self.robot_width = self.setupParam('~robot_width', params.robot_width)
		self.left_turn_thresh = self.setupParam('~left_turn_thresh', -0.17)
		self.right_turn_thresh = self.setupParam('~right_turn_thresh', 0.2)
		self.left_turn_std_ratio = self.setupParam('~left_turn_std_ratio', 1.5)
		self.right_turn_std_ratio = self.setupParam('~right_turn_std_ratio', 1.3)
		self.white_dir_correction = self.setupParam('~white_dir_correction', params.white_dir_correction)
		self.yellow_dir_correction = self.setupParam('~yellow_dir_correction', params.yellow_dir_correction)

		# Node attributes
		self.gearbox = Gearbox(v_min=self.v_min, v_max=self.v_max, dv_neg=self.dv_neg, dv_pos=self.dv_pos, w_gain_min=self.w_gain_min, w_gain_max=self.w_gain_max, func=self.func)
		self.avg_white_buffer = Buffer(5)
		self.avg_yellow_buffer = Buffer(5)
		self.turn_state = 'straight'
		self.init_v = self.v_min
		self.prev_target_point = np.array([1., 0.])
		self.obslist_msg = None
		self.detection_msg = False
		self.detection_list = []
		
		# Publishers
		self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
		self.pub_actuator_limits_received = rospy.Publisher("~actuator_limits_received", BoolStamped, queue_size=1)
		if self.verbose:
			self.bridge = CvBridge()
			self.pub_path_points = rospy.Publisher("~path_points", Image, queue_size=1)
			self.ground_image = None
			self.vis_scale = 1.5

		# Subscribers
		self.sub_filtered_lines = rospy.Subscriber("~seglist_filtered", SegmentList, self.updateFilteredLineSeg, queue_size=1)
		self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed", WheelsCmdStamped, self.updateWheelsCmdExecuted, queue_size=1)
		self.sub_actuator_limits = rospy.Subscriber("~actuator_limits", Twist2DStamped, self.updateActuatorLimits, queue_size=1)
		if self.vehicle_avoidance:
			self.sub_obslist_ = rospy.Subscriber("~obslist", ObstacleProjectedDetectionList, self.obslist_cb)

		# safe shutdown
		rospy.on_shutdown(self.custom_shutdown)

		rospy.loginfo("[%s] Initialized " % (rospy.get_name()))
		
		rospy.Timer(rospy.Duration.from_sec(2.0), self.updateParams)

	def obslist_cb(self, obslist_msg):
		self.obslist_msg = obslist_msg
		
		self.detection_list = [[0, 0, 0, 0] for _ in range(len(self.obslist_msg.list) / 4)]
		count_ind_list = [0 for _ in range(len(self.obslist_msg.list) / 4)]
		for detection in self.obslist_msg.list:
			x = float(detection.location.x)
			y = float(detection.location.y)
			ind = int(detection.type.type / 4)
			loc = int(detection.type.type % 4)
			self.detection_list[ind][loc] = [x, y]
			count_ind_list[ind] += 1

	def updateParams(self, _event):
		# Updated parameters
		changed_params = []
		for param_name in self.rosparamlist:
			old_val = getattr(self, param_name)
			new_val = rospy.get_param('~' + param_name)
			if old_val != new_val:
				changed_params.append(param_name)
				setattr(self, param_name, new_val)
		
		if len(changed_params) > 0:
			self.loginfo('*** Updated Parameters ***')
			for param_name in changed_params:
				self.loginfo('~%s: %s' % (str(param_name), getattr(self, param_name)))

		# Verbose update
		if 'verbose' in changed_params:
			self.loginfo('Verbose is now %r' % self.verbose)
			if self.verbose:
				self.bridge = CvBridge()
				self.pub_path_points = rospy.Publisher("~path_points", Image, queue_size=1)
				self.ground_image = None
				self.vis_scale = 1.5
		
		# Vehicle avoidance update
		if 'vehicle_avoidance' in changed_params:
			self.loginfo('vehicle_avoidance is now %r' % self.vehicle_avoidance)
			if self.vehicle_avoidance:
				self.sub_obslist_ = rospy.Subscriber("~obslist", ObstacleProjectedDetectionList, self.obslist_cb)
		
		# Gearbox update
		if ('v_min' in changed_params) or ('v_max' in changed_params) or ('dv_neg' in changed_params) or ('dv_pos' in changed_params) or ('w_gain_min' in changed_params) or ('w_gain_max' in changed_params) or ('func' in changed_params):
			self.gearbox = Gearbox(v_min=self.v_min, v_max=self.v_max, dv_neg=self.dv_neg, dv_pos=self.dv_pos, w_gain_min=self.w_gain_min, w_gain_max=self.w_gain_max, func=self.func)

	def setupParam(self, param_name, default_value):
		# value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, default_value)
		rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, default_value))
		if param_name not in self.rosparamlist:
			self.rosparamlist.append(param_name[1:])
		return default_value

	def updateFilteredLineSeg(self, gp_segment_list):
		# Stop duckiebot if it's too close to another vehicle
		if self.vehicle_avoidance:
			if self.collisionSafetyStop(gp_segment_list.header):
				return

		# Get points from SegmentList message and covert them to numpy
		white_points_p0, white_points_p1, yellow_points_p0, yellow_points_p1 = self.segmentUnpack(gp_segment_list)
		
		# Preprocess the points
		white_points, avg_abs_white_dir, num_white, min_val_w, max_val_w = self.preprocessPoints(white_points_p0, white_points_p1)
		yellow_points, avg_abs_yellow_dir, num_yellow, min_val_y, max_val_y = self.preprocessPoints(yellow_points_p0, yellow_points_p1)

		# Compute path points
		white_path_points = self.computePathPoints(white_points, num_white, avg_abs_white_dir, self.white_dir_correction, color='white')
		yellow_path_points = self.computePathPoints(yellow_points, num_yellow, avg_abs_yellow_dir, self.yellow_dir_correction, color='yellow')
		
		# Check if we're close to a turn and update gearbox state
		self.turn_state = self.turnDetection(white_points, yellow_points, num_white, num_yellow)
		self.updateGearbox(following_yellow=(num_yellow > 1))
		
		# Compute pure pursuit target point
		target_point = self.computeTargetPoint(yellow_path_points, white_path_points)
		
		# Compute v and omega
		target_point = target_point * self.lookahead_distance / np.linalg.norm(target_point)
		v, omega = self.pure_pursuit(target_point, np.array([0, 0]), np.pi / 2, follow_dist=self.lookahead_distance)
		
		# Publish wheel commands
		car_cmd_msg = Twist2DStamped()
		car_cmd_msg.header = gp_segment_list.header
		car_cmd_msg.v = v
		car_cmd_msg.omega = omega
		self.publishCmd(car_cmd_msg)

		# Publish visualizations
		if self.verbose:
			min_val = np.minimum(min_val_y, min_val_w) if (np.minimum(min_val_y, min_val_w) < 1e3) else None
			max_val = np.maximum(max_val_y, max_val_w) if (np.maximum(max_val_y, max_val_w) > -1e3) else None
			ground_image = drawImage(target_point, white_path_points, yellow_path_points, self.detection_list, min_val, max_val, self.vis_scale, self.robot_width, self.obs_stop_thresh)
			image_msg_out = self.bridge.cv2_to_imgmsg(ground_image, "bgr8")
			self.pub_path_points.publish(image_msg_out)

	def collisionSafetyStop(self, msg_header=None):
		for tl, tr, br, bl in self.detection_list:
			if ((bl[0] > 0) and (bl[0] < self.obs_stop_thresh)) or ((br[0] > 0) and (br[0] < self.obs_stop_thresh)):
				robot_occ = [-self.robot_width / 2, self.robot_width / 2]
				rightBound = np.maximum(-self.robot_width, br[1])
				leftBound = np.minimum(self.robot_width, bl[1])
				# self.loginfo('bl: %s | br: %s' % (str(bl), str(br)))
				# self.loginfo('tl: %s | tr: %s' % (str(tl), str(tr)))
				# self.loginfo('leftBound - rightBound: %s' % str(leftBound - rightBound))
				if (leftBound - rightBound) > 0:
					car_cmd_msg = Twist2DStamped()
					if msg_header is not None:
						car_cmd_msg.header = msg_header
					car_cmd_msg.v = 0
					car_cmd_msg.omega = 0
					self.publishCmd(car_cmd_msg)
					for _ in range(10):
						self.gearbox.down()
					return True
		return False

	def segmentUnpack(self, gp_segment_list):
		white_points_p0 = []
		white_points_p1 = []
		yellow_points_p0 = []
		yellow_points_p1 = []

		if gp_segment_list is not None:
			for segment in gp_segment_list.segments:
				color = segment.color
				p0 = segment.points[0]
				p1 = segment.points[1]
				p0 = [p0.x, p0.y]
				p1 = [p1.x, p1.y]
				if (color == Segment.WHITE):
					white_points_p0.append(p0)
					white_points_p1.append(p1)
				elif (color == Segment.YELLOW):
					yellow_points_p0.append(p0)
					yellow_points_p1.append(p1)
				else:
					pass
		
		white_points_p0 = np.array(white_points_p0, ndmin=2)
		white_points_p1 = np.array(white_points_p1, ndmin=2)
		yellow_points_p0 = np.array(yellow_points_p0, ndmin=2)
		yellow_points_p1 = np.array(yellow_points_p1, ndmin=2)
		return white_points_p0, white_points_p1, yellow_points_p0, yellow_points_p1

	def preprocessPoints(self, points_p0, points_p1):
		points = None
		avg_abs_dir = None
		min_val = 1e5
		max_val = -1e5
		num_points = points_p0.shape[0] if (points_p0.shape[1] == 2) else 0
		if (num_points > 1):
			points_keep = np.linalg.norm(points_p0, axis=1) < 0.7
			if np.sum(points_keep) == 0:
				points_keep = (np.ones_like(points_keep) > 0)
			num_points = np.sum(points_keep)
			points_p0 = points_p0[points_keep]
			points_p1 = points_p1[points_keep]
			points = np.vstack([points_p0, points_p1])
			t = points_p1 - points_p0
			t_normalized = t / np.linalg.norm(t, axis=1, keepdims=True)
			avg_abs_dir = np.mean(np.abs(t_normalized), axis=0)
			min_val = np.min(points)
			max_val = np.max(points)
		return points, avg_abs_dir, num_points, min_val, max_val
	
	def computePathPoints(self, points, num_points, avg_abs_dir, dir_correction, color):
		path_points = None
		if num_points > 1:
			avg_dir_normalized = avg_abs_dir / np.linalg.norm(avg_abs_dir)
			h_offset = 1. if (color == 'white') else -1.
			offset_dir = np.array([-avg_dir_normalized[1] * dir_correction, h_offset])
			offset = 0.5 * self.lane_width * offset_dir
			path_points = points + offset
		return path_points

	def turnDetection(self, white_points, yellow_points, num_white, num_yellow):
		turn_state = 'straight'

		if num_white > 1:
			avg_white_point = np.mean(white_points, axis=0)
			self.avg_white_buffer.push(avg_white_point[1])
			std_ratio = np.std(white_points[:,0] / np.std(white_points[:,1]))
			left_turn_detected = (self.avg_white_buffer.mean() > self.left_turn_thresh) and (std_ratio < self.left_turn_std_ratio)
			if left_turn_detected:
				turn_state = 'left'
				# self.loginfo('LEFT TURN! %s | %s' % (str(self.avg_white_buffer.mean()), str(std_ratio)))
		else:
			self.avg_white_buffer.pop()
			
		if num_yellow > 1:
			avg_yellow_point = np.mean(yellow_points, axis=0)
			self.avg_yellow_buffer.push(avg_yellow_point[1])
			std_ratio = np.std(yellow_points[:,0] / np.std(yellow_points[:,1]))
			right_turn_detected = (self.avg_yellow_buffer.mean() < self.right_turn_thresh) and (std_ratio < self.right_turn_std_ratio)
			if right_turn_detected:
				turn_state = 'right'
				# self.loginfo('RIGHT TURN! %s | %s' % (str(self.avg_yellow_buffer.mean()), str(std_ratio)))
		else:
			self.avg_yellow_buffer.pop()
		
		# if turn_state == 'straight':
		# 	self.loginfo('-')

		return turn_state
	
	def updateGearbox(self, following_yellow):
		if self.turn_state == 'right':
			for _ in range(self.num_r_vdown):
				self.gearbox.down()

		elif self.turn_state == 'left':
			self.gearbox.down()

		elif self.turn_state == 'straight':
			if following_yellow:
				for _ in range(self.num_y_vup):
					self.gearbox.up()
			else:
				self.gearbox.up()

	def computeTargetPoint(self, yellow_path_points, white_path_points):
		if yellow_path_points is not None:
			target_point = np.mean(yellow_path_points, axis=0)
			self.prev_target_point = target_point
		elif white_path_points is not None:
			target_point = np.mean(white_path_points, axis=0)
			self.prev_target_point = target_point
		else:
			target_point = self.prev_target_point
		return target_point

	def updateActuatorLimits(self, msg_actuator_limits):
		self.actuator_limits = msg_actuator_limits
		rospy.logdebug("actuator limits updated to: ")
		rospy.logdebug("actuator_limits.v: " + str(self.actuator_limits.v))
		rospy.logdebug("actuator_limits.omega: " + str(self.actuator_limits.omega))
		msg_actuator_limits_received = BoolStamped()
		msg_actuator_limits_received.data = True
		self.pub_actuator_limits_received.publish(msg_actuator_limits_received)
	
	def updateWheelsCmdExecuted(self, msg_wheels_cmd):
		self.wheels_cmd_executed = msg_wheels_cmd

	def _vec2angle(self, vector):
		return np.arctan2(vector[0], vector[1])

	def _mat2angle(self, mat):
		return np.arctan2(mat[:, 0], mat[:, 1])

	def pure_pursuit(self, curve_point, pos, angle, follow_dist=0.25):
		omega = 0.
		v = self.gearbox.v
		if curve_point is not None:
			path_dir = curve_point - pos
			path_dir /= np.linalg.norm(path_dir)
			alpha = angle - self._vec2angle(path_dir)
			# Increase w_gain at right turns
			if (self.gearbox.wg == self.gearbox.w_gain_max) and (self.turn_state == 'right'):
				self.gearbox.wg *= self.right_wg_scale
			omega = self.gearbox.wg * 2 * v * np.sin(alpha) / follow_dist
		return v, omega
		
	def custom_shutdown(self):
		rospy.loginfo("[%s] Shutting down..." % self.node_name)

		# Stop the duckie
		car_cmd_msg = Twist2DStamped()
		car_cmd_msg.v = 0
		car_cmd_msg.omega = 0
		self.publishCmd(car_cmd_msg)
		
		# Stop listening
		self.sub_filtered_lines.unregister()

		rospy.sleep(3)    #To make sure that it gets published.
		rospy.loginfo("[%s] Shutdown" % self.node_name)
	
	def loginfo(self, s):
		rospy.loginfo('[%s] %s' % (self.node_name, s))
	
	def publishCmd(self, car_cmd_msg):
		self.pub_car_cmd.publish(car_cmd_msg)
		
if __name__ == "__main__":

	rospy.init_node("pure_pursuit_node", anonymous=False)

	pure_pursuit = pure_pursuit()
	rospy.spin()