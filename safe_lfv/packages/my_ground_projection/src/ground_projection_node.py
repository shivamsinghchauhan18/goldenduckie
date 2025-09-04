#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import (Segment, SegmentList, ObstacleImageDetection, ObstacleImageDetectionList, ObstacleProjectedDetection, ObstacleProjectedDetectionList, Vector2D)
import duckietown_utils as dtu
from ground_projection.ground_projection_interface import GroundProjection, \
	get_ground_projection_geometry_for_robot
from ground_projection.srv import EstimateHomography, EstimateHomographyResponse, GetGroundCoord, GetGroundCoordResponse, GetImageCoord, GetImageCoordResponse  #@UnresolvedImport
import rospy
from sensor_msgs.msg import (Image, CameraInfo)
import copy
from std_msgs.msg import Float32

class GroundProjectionNode(object):

	def __init__(self):
		self.node_name = "Ground Projection"
		self.active = True
		self.bridge = CvBridge()

		robot_name = rospy.get_param("~config_file_name", None)

		if robot_name is None:
			robot_name = dtu.get_current_robot_name()

		self.robot_name = robot_name

		self.gp = GroundProjection(self.robot_name)

		self.gpg = get_ground_projection_geometry_for_robot(self.robot_name)



		self.image_channel_name = "image_raw"

		# Subs and Pubs
		self.pub_lineseglist_ = rospy.Publisher("~lineseglist_out", SegmentList, queue_size=1)
		self.sub_lineseglist_ = rospy.Subscriber("~lineseglist_in", SegmentList, self.lineseglist_cb)

		self.pub_obslist_ = rospy.Publisher("~obslist_out", ObstacleProjectedDetectionList, queue_size=1)
		self.sub_obslist_ = rospy.Subscriber("~obslist_in", ObstacleImageDetectionList, self.obslist_cb)
		self.pub_time_elapsed = rospy.Publisher("~obs_projection_time", Float32, queue_size=1)

		# TODO prepare services
		self.service_homog_ = rospy.Service("~estimate_homography", EstimateHomography, self.estimate_homography_cb)
		self.service_gnd_coord_ = rospy.Service("~get_ground_coordinate", GetGroundCoord, self.get_ground_coordinate_cb)
		self.service_img_coord_ = rospy.Service("~get_image_coordinate", GetImageCoord, self.get_image_coordinate_cb)
		self.loginfo('############################')
		self.loginfo('### MY_GROUND_PROJECTION ###')
		self.loginfo('############################')
		self.loginfo('Using my_ground_projection')


	def rectifyImage(self, img_msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")
		except CvBridgeError as e:
			dtu.logger.error(e)
		return self.gpg.rectify(cv_image)

	def lineseglist_cb(self, seglist_msg):
		seglist_out = SegmentList()
		seglist_out.header = seglist_msg.header
		for received_segment in seglist_msg.segments:
			new_segment = Segment()
			new_segment.points[0] = self.gpg.vector2ground(received_segment.pixels_normalized[0])
			new_segment.points[1] = self.gpg.vector2ground(received_segment.pixels_normalized[1])
			new_segment.color = received_segment.color
			# TODO what about normal and points
			seglist_out.segments.append(new_segment)
		self.pub_lineseglist_.publish(seglist_out)
	
	def obslist_cb(self, obslist_msg):
		# if obslist_msg is None:
		#     return
		# if len(obslist_msg.list) == 0:
		#     return
		start = rospy.Time.now()
		obslist_out = ObstacleProjectedDetectionList()
		obslist_out.header = obslist_msg.header
		imwidth = obslist_msg.imwidth
		imheight = obslist_msg.imheight

		gp_detectionlist = []
		ind = 0
		for detection in obslist_msg.list:
			gp_detection = ObstacleProjectedDetection()
			x0 = float(detection.bounding_box.x) / imwidth
			y0 = float(detection.bounding_box.y) / imheight
			w = float(detection.bounding_box.w) / imwidth
			h = float(detection.bounding_box.h) / imheight
			x1 = x0 + w
			y1 = y0 + h

			box_corners = ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
			for p in box_corners:
				vector = Vector2D()
				vector.x = p[0]
				vector.y = p[1]
				gp_vector = self.gpg.vector2ground(vector)
				gp_x, gp_y = (gp_vector.x, gp_vector.y)
				gp_detection.location.x = gp_x
				gp_detection.location.y = gp_y
				gp_detection.type.type = ind
				gp_detectionlist.append(copy.deepcopy(gp_detection))
				ind += 1

		obslist_out.header.stamp = rospy.Time.now()
		obslist_out.list = gp_detectionlist
		self.pub_obslist_.publish(obslist_out)
		elapsed_time = (rospy.Time.now() - start).to_sec()
		self.pub_time_elapsed.publish(elapsed_time)

	def get_ground_coordinate_cb(self, req):
		return GetGroundCoordResponse(self.gpg.vector2ground(req.normalized_uv))

	def get_image_coordinate_cb(self, req):
		return GetImageCoordResponse(self.gpg.ground2pixel(req.gp))

	def estimate_homography_cb(self, req):
		rospy.loginfo("Estimating homography")
		rospy.loginfo("Waiting for raw image")
		img_msg = rospy.wait_for_message("/" + self.robot_name + "/camera_node/image/raw", Image)
		rospy.loginfo("Got raw image")
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)
		self.gp.estimate_homography(cv_image)
		rospy.loginfo("wrote homography")
		return EstimateHomographyResponse()

	def onShutdown(self):
		rospy.loginfo("[GroundProjectionNode] Shutdown.")
	
	def loginfo(self, s):
		rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
	rospy.init_node('ground_projection', anonymous=False)
	ground_projection_node = GroundProjectionNode()
	rospy.on_shutdown(ground_projection_node.onShutdown)
	rospy.spin()
