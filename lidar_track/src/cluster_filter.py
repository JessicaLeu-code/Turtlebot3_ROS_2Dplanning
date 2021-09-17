#!/usr/bin/env python
import rospy
import numpy as np

from util import *

from research.msg import Polygons
from geometry_msgs.msg import Twist, PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PolygonStamped, Point32, Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



class Sim(object):
	def __init__(self):
		rospy.init_node('filter')	
		rate = rospy.Rate(10)
		wall_viz = rospy.Publisher('/wall_viz', MarkerArray, queue_size=1)
		pub_dict = {
			0 : rospy.Publisher('/line_0_tagged', Marker, queue_size=1),
			1 : rospy.Publisher('/line_1_tagged', Marker, queue_size=1),
			2 : rospy.Publisher('/line_2_tagged', Marker, queue_size=1),
			3 : rospy.Publisher('/line_3_tagged', Marker, queue_size=1),
			4 : rospy.Publisher('/line_4_tagged', Marker, queue_size=1),
			5 : rospy.Publisher('/line_5_tagged', Marker, queue_size=1)
		}
		self.prob = list(list() for i in range(6))
		visual = MarkerArray()
		visual.markers = [Marker() for i in range(6)]
		self.x = [0,0,0]

		def odom_callback(msg):
			dd,asd,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
			self.x = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
			#print("turtlebot ODOM", self.x)

		# NOTE: odom might need to be remapped when working on actual robot
		rospy.Subscriber('/odom', Odometry, odom_callback)

		def line_callback0(lidar_data):
			find_lines_and_pub(lidar_data, 0)

		def line_callback1(lidar_data):
			find_lines_and_pub(lidar_data, 1)

		def line_callback2(lidar_data):
			find_lines_and_pub(lidar_data, 2)

		def line_callback3(lidar_data):
			find_lines_and_pub(lidar_data, 3)

		def line_callback4(lidar_data):
			find_lines_and_pub(lidar_data, 4)

		def line_callback5(lidar_data):
			find_lines_and_pub(lidar_data, 5)

		def find_lines_and_pub(lidar_data, clusternum):
			#lines, img = find_lines(lidar_data, self.prev_real_lines, self.x)

			# apply transformation
			# try:
			# 	pos, qu = self.tf.lookupTransform("/odom", "/base_scan", rospy.Time())
			# 	#print("TRANS", pos, qu)
			# 	t_mat = self.tf.fromTranslationRotation(pos, qu)

			pub = pub_dict[clusternum]
			n = len(lidar_data.points)
			pts = []
			
			if n == 0:
				lines = None
			else:
				p_mat = np.ones((4, n))
				for i in range(n):
					pt = lidar_data.points[i]
					p_mat[0, i] = pt.x
					p_mat[1, i] = pt.y
					p_mat[2, i] = 0
				
				p_trans = p_mat #np.dot(t_mat, p_mat)

				lines = set()
				for i in range(n/2):
					line = ((p_trans[0, 2*i], p_trans[1, 2*i]), (p_trans[0, 2*i+1], p_trans[1, 2*i+1]))
					lines.add(line)

				pt = np.mean(p_trans, axis=1)
				y = Point()
				y.x = pt[0]
				y.y = pt[1]
				pts = [y]

			obst = cluster_filter(lines, self.x[0:2])

			#con = con + 1

			if obst:
				print("OBJECT")
				self.prob[clusternum].append(1)	
				if len(self.prob[clusternum]) > 16:
					self.prob[clusternum].pop(0)
			else:
				print("WALL")
				self.prob[clusternum].append(0)
				if len(self.prob[clusternum]) > 16:
					self.prob[clusternum].pop(0)

			summ = 0.0
			for f in self.prob[clusternum]:
				summ += f

			p = summ / len(self.prob[clusternum])

			# prob = self.obj / (self.obj + self.wall)
			print("Likelihood of obj", p)

			if p >= 0.5:
				# marker = Marker()
				# marker.header.frame_id = "/base_scan"
				# marker.id = clusternum
				# marker.type = marker.POINTS
				# marker.scale.x = 0.5
				# marker.scale.y = 0.5
				# marker.scale.z = 0.5
				# marker.color.b = 1.0
				# marker.color.a = 1.0
				# marker.points = pts
				# #visual.markers.append(marker)
				# visual.markers[clusternum] = marker
				# wall_viz.publish(visual)
				pub.publish(lidar_data)
			else:
				marker = Marker()
				marker.header.frame_id = "/base_scan"
				marker.id = clusternum
				marker.type = marker.POINTS
				marker.scale.x = 0.5
				marker.scale.y = 0.5
				marker.scale.z = 0.5
				marker.color.g = 1.0
				marker.color.a = 1.0
				marker.points = pts
				#visual.markers.append(marker)
				visual.markers[clusternum] = marker
				wall_viz.publish(visual)
			

		#rospy.Subscriber('/publish_line_markers', Marker, find_lines_and_pub)

		rospy.Subscriber('/line_0', Marker, line_callback0)
		rospy.Subscriber('/line_1', Marker, line_callback1)
		rospy.Subscriber('/line_2', Marker, line_callback2)
		rospy.Subscriber('/line_3', Marker, line_callback3)
		rospy.Subscriber('/line_4', Marker, line_callback4)
		rospy.Subscriber('/line_5', Marker, line_callback5)





		rospy.spin()



if __name__ == '__main__':
    try:
		Sim()
		rospy.spin()
    except rospy.ROSInterruptException:
		pass
