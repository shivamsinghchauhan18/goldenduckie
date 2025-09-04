#!/usr/bin/env python

import rospy
import math
import time
from collections import deque
from duckietown_msgs.msg import ObstacleProjectedDetection, ObstacleProjectedDetectionList


class ExpSmoother(object):
    def __init__(self, alpha=0.5):
        self.alpha = alpha
        self.has_init = False
        self.x = 0.0
        self.y = 0.0

    def update(self, x, y):
        if not self.has_init:
            self.x, self.y = x, y
            self.has_init = True
        else:
            a = self.alpha
            self.x = a * x + (1 - a) * self.x
            self.y = a * y + (1 - a) * self.y
        return self.x, self.y


class Track(object):
    def __init__(self, track_id, centroid, corners, timestamp, alpha):
        self.id = track_id
        self.smoother = ExpSmoother(alpha)
        self.cx, self.cy = self.smoother.update(centroid[0], centroid[1])
        self.last_time = timestamp
        self.hits = 1
        # store relative corner vectors from centroid
        self.rel = self._relative_corners(centroid, corners)

    def _relative_corners(self, centroid, corners):
        cx, cy = centroid
        rel = []
        for (x, y) in corners:
            rel.append((x - cx, y - cy))
        return rel

    def distance(self, centroid):
        x, y = centroid
        return math.hypot(self.cx - x, self.cy - y)

    def update(self, centroid, corners, stamp):
        self.cx, self.cy = self.smoother.update(centroid[0], centroid[1])
        self.last_time = stamp
        self.hits += 1
        if corners:
            self.rel = self._relative_corners(centroid, corners)


class ObstacleTrackingNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Parameters
        self.alpha = rospy.get_param('~alpha', 0.5)  # smoothing factor
        self.association_radius = rospy.get_param('~association_radius', 0.15)  # meters
        self.max_age = rospy.get_param('~max_age', 1.0)  # seconds without updates
        self.min_hits = rospy.get_param('~min_hits', 1)

        self.tracks = {}
        self.next_id = 0

        # IO
    self.sub = rospy.Subscriber('~obslist_in', ObstacleProjectedDetectionList, self.cb_obslist, queue_size=1)
        self.pub = rospy.Publisher('~obslist_tracked', ObstacleProjectedDetectionList, queue_size=1)

        rospy.loginfo('[%s] obstacle tracking initialized (alpha=%.2f, radius=%.2f, max_age=%.2f, min_hits=%d)'
                      % (self.node_name, self.alpha, self.association_radius, self.max_age, self.min_hits))

        self.timer = rospy.Timer(rospy.Duration(0.5), self.prune_old_tracks)

    def prune_old_tracks(self, _evt):
        now = rospy.Time.now()
        to_delete = []
        for tid, t in self.tracks.items():
            if (now - t.last_time).to_sec() > self.max_age:
                to_delete.append(tid)
        for tid in to_delete:
            del self.tracks[tid]

    def assign(self, observations, stamp):
        # Greedy nearest-neighbor association on obstacle centroids
        for obs in observations:
            centroid = obs['centroid']
            corners = obs['corners']  # list of 4 tuples
            best_id = None
            best_dist = float('inf')
            for tid, t in self.tracks.items():
                d = t.distance(centroid)
                if d < best_dist:
                    best_dist = d
                    best_id = tid

            if best_dist <= self.association_radius and best_id is not None:
                self.tracks[best_id].update(centroid, corners, stamp)
            else:
                tid = self.next_id
                self.next_id += 1
                self.tracks[tid] = Track(tid, centroid, corners, stamp, self.alpha)

    def cb_obslist(self, msg):
        # Group incoming corners by obstacle index (type.type // 4)
        groups = {}
        for det in msg.list:
            idx = int(det.type.type) // 4
            corner_idx = int(det.type.type) % 4
            if idx not in groups:
                groups[idx] = [None, None, None, None]
            groups[idx][corner_idx] = (det.location.x, det.location.y)

        # Build observations for complete quads
        observations = []
        for idx, corners in groups.items():
            if any(c is None for c in corners):
                continue
            xs = [c[0] for c in corners]
            ys = [c[1] for c in corners]
            centroid = (sum(xs) / 4.0, sum(ys) / 4.0)
            observations.append({'centroid': centroid, 'corners': corners})

        self.assign(observations, msg.header.stamp)

        # Build output list
        out = ObstacleProjectedDetectionList()
        out.header = msg.header
        out.list = []

        for tid, t in self.tracks.items():
            if t.hits < self.min_hits or not t.rel:
                continue
            # Reconstruct 4-corner representation
            for k, (rx, ry) in enumerate(t.rel):
                d = ObstacleProjectedDetection()
                d.location.x = t.cx + rx
                d.location.y = t.cy + ry
                d.location.z = 0.0
                d.type.type = int(tid) * 4 + k
                out.list.append(d)

        self.pub.publish(out)

    def on_shutdown(self):
        rospy.loginfo('[%s] Shutting down.' % self.node_name)


if __name__ == '__main__':
    rospy.init_node('obstacle_tracking_node', anonymous=False)
    node = ObstacleTrackingNode()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
