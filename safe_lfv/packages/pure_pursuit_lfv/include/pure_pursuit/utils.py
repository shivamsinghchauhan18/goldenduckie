#!/usr/bin/env python
import math
import numpy as np
import cv2
import rospy

class Gearbox(object):
	def __init__(self, v_min, v_max, dv_neg, dv_pos, w_gain_min, w_gain_max, func='x'):
		"""
		Gearbox intuition:
		As we speed up, we decrease omega gain to avoid zig zag movement.
		As we slow down, we increase omega gain because we often slow
		down when we need to make turns.
		"""
		self.v = v_min
		self.v_min = v_min
		self.v_max = v_max
		self.dv_neg = dv_neg
		self.dv_pos = dv_pos

		# We set dw such that when v=v_min, wg=w_gain_max; and when v=v_max, wg=w_gain_min
		# In other words from v_min to v_max takes same amount of time as from w_max to w_min.
		self.wg = w_gain_max
		self.w_gain_min = w_gain_min
		self.w_gain_max = w_gain_max
		wg_diff = w_gain_max - w_gain_min
		v_diff = v_max - v_min
		self.dw_neg = (-dv_pos / v_diff) * wg_diff
		self.dw_pos = (-dv_neg / v_diff) * wg_diff
		
		if func == 'x':
			f = lambda x: x
		elif func == 'x2':
			f = lambda x: (x ** 2)
		elif func == 'x3':
			f = lambda x: (x ** 3)
		elif func == '1-cosx':
			f = lambda x: 1 - np.cos(np.pi * x / 2)
		elif func == '2x':
			f = lambda x: (2 ** x) - 1
		elif func == 'expx':
			n = 3
			f = lambda x: ((n ** x) - 1) / (n - 1)
		
		self.f_v = lambda x: v_min + (f(x) * v_diff)
		self.f_wg = lambda x: w_gain_min + (f(x) * wg_diff)
		self.in_v = v_min
		self.in_wg = w_gain_max
		self.wg_diff = wg_diff
		self.v_diff = v_diff
		self.loginfo('Initialized Gearbox: v_min=%s, v_max=%s,  dv_neg=%s, dv_pos=%s, w_gain_min=%s, w_gain_max=%s, func=%s' % (str(v_min), str(v_max), str(dv_neg), str(dv_pos), str(w_gain_min), str(w_gain_max), str(func)))
	
	def up(self):
		self.in_v += self.dv_pos
		self.in_v = np.minimum(self.in_v, self.v_max)
		self.v = self.f_v(self.norm_v(self.in_v))
		
		self.in_wg += self.dw_neg
		self.in_wg = np.maximum(self.in_wg, self.w_gain_min)
		self.wg = self.f_wg(self.norm_wg(self.in_wg))
		
	def down(self):
		self.in_v += self.dv_neg
		self.in_v = np.maximum(self.in_v, self.v_min)
		self.v = self.f_v(self.norm_v(self.in_v))

		self.in_wg += self.dw_pos
		self.in_wg = np.minimum(self.in_wg, self.w_gain_max)
		self.wg = self.f_wg(self.norm_wg(self.in_wg))
	
	def norm_v(self, v):
		return (v - self.v_min) / self.v_diff
	
	def norm_wg(self, wg):
		return (wg - self.w_gain_min) / self.wg_diff

	def loginfo(self, s):
		rospy.loginfo('[GEARBOX] %s' % s)


class Buffer(object):
	def __init__(self, size, init_val=None):
		self.size = size
		self.init_val = init_val
		if init_val is None:
			self.buffer = []
		else:
			self.buffer = [init_val] * size

	def push(self, x):
		self.buffer.append(x)
		if len(self.buffer) > self.size:
			self.buffer = self.buffer[1:]
	
	def pop(self):
		if len(self.buffer) > 0:
			self.buffer = self.buffer[1:]

	def mean(self):
		return np.mean(np.array(self.buffer))
	
	def sum(self):
		return np.sum(np.array(self.buffer))

	def tolist(self):
		return copy.deepcopy(self.buffer)
	
	def reset(self):
		if self.init_val is None:
			self.buffer = []
		else:
			self.buffer = [self.init_val] * self.size


def drawImage(target_point, white_path_points, yellow_path_points, detection_list, min_val, max_val, vis_scale, robot_width, obs_stop_thresh, img_size=480):
	if (min_val is not None) and (not math.isnan(min_val)):
		vis_scale = np.maximum(vis_scale, np.abs(min_val))
	if (max_val is not None) and (not math.isnan(max_val)):
		vis_scale = np.maximum(vis_scale, np.abs(max_val))
	ground_image = np.zeros((img_size,img_size,3), np.uint8)
	min_val = -vis_scale
	max_val = vis_scale
	
	for p0, p1, p2, p3 in detection_list:
		i0, j0 = point2pixel(p0, img_size, min_val, max_val)
		i1, j1 = point2pixel(p1, img_size, min_val, max_val)
		i2, j2 = point2pixel(p2, img_size, min_val, max_val)
		i3, j3 = point2pixel(p3, img_size, min_val, max_val)
		pts = np.array([[j0, i0], [j1, i1], [j2, i2], [j3, i3]], np.int32)
		cv2.fillPoly(ground_image, [pts], (0, 0, 255))

	# Robot boundary box
	color = np.array([120, 120, 120])
	i0, j0 = point2pixel([obs_stop_thresh, robot_width / 2], img_size, min_val, max_val)
	i1, j1 = point2pixel([0, -robot_width / 2], img_size, min_val, max_val)
	ground_image[i0:i1, j0-1:j0+1, :] = color
	ground_image[i0:i1, j1-1:j1+1, :] = color
	ground_image[i0-1:i0+1, j0:j1, :] = color
	# ground_image[i1-1:i1+1, j0:j1, :] = color

	# Target point
	i, j = point2pixel(target_point, img_size, min_val, max_val)
	ground_image[i-3:i+3, j-3:j+3, :] = np.array([0, 255, 0])

	vectors = []
	num_white = white_path_points.shape[0] if ((white_path_points is not None) and (white_path_points.shape[1] == 2)) else 0
	num_yellow = yellow_path_points.shape[0] if ((yellow_path_points is not None) and (yellow_path_points.shape[1] == 2)) else 0
	if (num_white > 1):
		for point in white_path_points:
			i, j = point2pixel(point, img_size, min_val, max_val)
			ground_image[i-1:i+1, j-1:j+1] = 255

	if (num_yellow > 1):
		for point in yellow_path_points:
			i, j = point2pixel(point, img_size, min_val, max_val)
			ground_image[i-1:i+1, j-1:j+1, 1:] = 255

	# Current robot point
	color = np.array([255, 255, 0])
	i, j = point2pixel(np.array([0, 0]), img_size, min_val, max_val)
	ground_image[i-4:i+4, j-4:j+4, :] = color

	return ground_image


def point2pixel(point, img_size, min_val, max_val):
	i = img_size - int((point[0] - min_val) * img_size / (max_val - min_val))
	j = img_size - int((point[1] - min_val) * img_size / (max_val - min_val))
	i = np.clip(i, 1, img_size-1)
	j = np.clip(j, 1, img_size-1)
	return (i, j)