#!/usr/bin/env python
import rospy 
import pyclipper
import numpy as np
from lidar_track.msg import Polygons
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, PolygonStamped, Point32, Point, PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from cfs_module_barrier import Cfs

import time
import logging

def sim():
	rospy.init_node('optimization')
	o = Optimizer()
	c = Cfs(21, 0)


	logger = logging.getLogger("testtest")
	sh = logging.StreamHandler()
	logger.addHandler(sh)

	rospy.Subscriber('/odom', Odometry, o.odom_callback)
	rospy.Subscriber('/filteredGPS', Point, o.filter_callback)
	rospy.Subscriber('/table_poly', PolygonStamped, o.obstacle_poly_callback) # from fake_human nodes
	rospy.Subscriber('/table_v', Twist, o.obstacle_vel_callback)
	rospy.Subscriber('/obstacle2_poly', Polygons, o.lobstacle_poly_callback) # from lidar
	#rospy.Subscriber('/obstacle2_v', Twist, o.lobstacle_vel_callback)
	rospy.Subscriber('/human', Point32, o.human_callback)
	rospy.Subscriber('/goal_state', Point32, o.goal_callback)
	rospy.Subscriber('follower_path', Path, o.follow_path_callback)
	rospy.Subscriber('obstacle2_v', Twist, o.human_Twist_callback)

	path_pub = rospy.Publisher('/path', Path, queue_size=100)
	margin_rviz_pub = rospy.Publisher('/margins', PolygonStamped, queue_size=100)
	goal_pub = rospy.Publisher('/goalrviz', PointStamped, queue_size=100)
	line_viz = rospy.Publisher('/line', Marker, queue_size=50)


	rate = rospy.Rate(10)
	try:
		barrier = rospy.get_param('~barrier_radius')
	except:
		barrier = 1.5
		
	c.set_barrier(barrier)


	while not rospy.is_shutdown():

		o.move_pred()
		c.set_margin(o.margin)

		obstacles = o.obstacles + o.lidar_obs
		vels = o.vels + o.lidar_vels

		#logger.info(o.human[2])

		if(np.sum(o.human)!=0):
			refpath, linegen = c.optimize(obstacles, vels, o.x, o.goal, o.f_waypoints, human=o.human) # o.human if tracking human
		else:
			refpath = [0]
			linegen = [0,0]

		marker = Marker()
		marker.header.frame_id = "/odom"
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD
		marker.scale.x = 0.03
		marker.scale.y = 0.03
		marker.scale.z = 0.03
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0
		pts = []
		for i in range(-10,10):
			add = Point()
			add.x = i
			add.y = linegen[0]*i+linegen[1]
			pts.append(add)
		marker.points = pts
		line_viz.publish(marker)



		if stopCheck(refpath, 0.2):
			path = to_path(refpath)
			path_pub.publish(path)
		else:
			path = to_path([])
			path_pub.publish(path)

		#elapsed_time = time.time() - start_time
		#logger.info("path_pub.publish(path)")
		#logger.info(elapsed_time)


		margin = o.get_margin_polygon()

		margin_rviz_pub.publish(margin)
		goal_pub.publish(o.get_goal())

		#elapsed_time = time.time() - start_time
		#logger.info("margin")
		#logger.info(elapsed_time)

		#logger.info("-----------------")

		rate.sleep()


# publish path only if length of the path is greater than threshold
def stopCheck(refpath, thres):
	prevPt = None
	dist = 0.0
	for i in range(len(refpath)/2):
		stPt = np.array([refpath[2*i], refpath[2*i+1]])
		if not prevPt is None:
			dist += np.linalg.norm(stPt - prevPt)
		prevPt = stPt
		if dist > thres:
			return True

	if dist < thres:
		return False
		
	return True


def to_path(refpath):
	poseArray = []

	for i in range(len(refpath)/2):
		poses = PoseStamped()
		poses.pose.position.x = refpath[2*i]
		poses.pose.position.y = refpath[2*i+1]
		poseArray += [poses]

	waypoints = Path()
	waypoints.header.frame_id = 'odom'
	waypoints.poses = poseArray
	return waypoints


class Optimizer:
	def __init__(self):
		self.x = [0,0,0,0,0]
		self.margin = 0
		self.obstacles = []
		self.lidar_obs = []
		self.vels = []
		self.lidar_vels = []
		self.hobs = [[],[]]
		self.pobs = []              # 
		self.gamma = 0.7            # discount constant
		self.p_stop = 0.5
		self.p_go = 0.5
		self.xyz = np.zeros(3)
		self.goal = [2, -4]
		self.human = [0,0,0,0]
		self.f_waypoints = []

	# checks if lidar object (human) is coming toward robot. if so, do not stop robot
	def incomingCheck(self):
		return

	def odom_callback(self, msg):
		_,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
		self.x = [msg.pose.pose.position.x,
				  msg.pose.pose.position.y,
				  yaw,
				  msg.twist.twist.linear.x,
				  msg.twist.twist.angular.z]

		# Uncomment to run experiment
		#self.x[0] = self.xyz[0]
		#self.x[1] = self.xyz[1]

	def dist(self):
		return np.linalg.norm(np.array(self.x[0:2]) - np.array(self.goal))

	def filter_callback(self, msg):
		self.xyz[0] = msg.x
		self.xyz[1] = msg.y
		self.xyz[2] = msg.z

	#test for obs detection
	def goal_callback(self, msg):
		self.goal[0] = msg.x
		self.goal[1] = msg.y

	def human_callback(self, msg):
		#if not self.human:
		#	self.human = [msg.x, msg.y]
		#	return

		self.human[0] = msg.x
		self.human[1] = msg.y

	def human_Twist_callback(self,msg):
		self.human[2] = msg.linear.x
		self.human[3] = msg.linear.y

	def get_goal(self):
		g = PointStamped()
		g.header.frame_id = 'odom'
		g.point.x = self.goal[0]
		g.point.y = self.goal[1]
		return g

	def obstacle_poly_callback(self, msg):
		pts = [[], []]

		for i in range(len(msg.polygon.points)):
			pts = np.concatenate((pts, [[msg.polygon.points[i].x], [msg.polygon.points[i].y]]), axis=1)
		self.obstacles = [pts]
		self.hobs = np.concatenate((self.hobs, np.mean(pts, axis=1).reshape(2,1)), axis=1)
		self.pobs = self.gamma*np.concatenate((self.pobs, [1]))
		self.pobs[-1] = 1
		self.pobs = self.pobs/sum(self.pobs)

	def obstacle_vel_callback(self, msg):
		vel = np.array([[msg.linear.x], [msg.linear.y]])
		self.vels = [vel]


	def lobstacle_poly_callback(self, msg):

		self.lidar_obs = []
		self.lidar_vels = []
		for j in range(len(msg.polygons)):
			pts = [[], []]
			for i in range(len(msg.polygons[j].polygon.points)):
				pts = np.concatenate((pts, [[msg.polygons[j].polygon.points[i].x], [msg.polygons[j].polygon.points[i].y]]), axis=1)
			self.lidar_obs.append(self.offset_polygon(pts, 0.4))

			vel = np.array([[msg.vels[j].linear.x], [msg.vels[j].linear.y]])
			self.lidar_vels.append(vel)



	def lobstacle_vel_callback(self, msg):
		vel = np.array([[msg.linear.x], [msg.linear.y]])
		self.lidar_vels = [vel]

	def follow_path_callback(self, msg):
		logger = logging.getLogger("testtest")
		sh = logging.StreamHandler()
		logger.addHandler(sh)

		self.f_waypoints = np.zeros([2,len(msg.poses)])

		for i in range(0, len(msg.poses)):
			self.f_waypoints[0,i] = msg.poses[i].pose.position.x
			self.f_waypoints[1,i] = msg.poses[i].pose.position.y

	def move_pred(self):
		r1 = 2
		r2 = 0.5

		if self.vels != [] and abs(self.vels[0][1]) > 0.04:
			self.p_go = self.p_go * r1
			self.p_stop = self.p_stop * r2 + 0.01
		else:
			self.p_go = self.p_go * r2 + 0.01
			self.p_stop = self.p_stop * r1

		self.p_go = self.p_go/(self.p_go + self.p_stop)

		if self.p_go > 0.5:
			self.margin = 0.3
		else:
			self.margin = self.get_margin()
			self.vels = [np.zeros((2,1))]

	def get_margin(self):
		if self.pobs == []:
			return 0.3
		omega = np.std(self.pobs[-10:] * self.hobs[:, -10:], axis=1)

		return 0.2+omega[1]

	def get_margin_polygon(self):
		if self.obstacles == []:
			return PolygonStamped()
		obstacle_margin = self.offset_polygon(self.obstacles[0], self.margin)
		return self.toPolygonStamped(obstacle_margin)

	def offset_polygon(self, polygon, margin):
		# obstacle = ()
		# for i in range(len(polygon[0])):
		#     obstacle = ((int(polygon[0,i]*100), int(polygon[1,i]*100)),) + obstacle
		# pco = pyclipper.PyclipperOffset()
		# pco.AddPath(obstacle, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
		# solution = pco.Execute(-margin)
		# solution = np.array(solution[0])/100.
		# return solution.T
		solution = np.array(polygon)
		for i in range(len(polygon[0])):
			if polygon[0,i] < np.average(polygon[0,:]):
				solution[0,i] -= margin 
			else:
				solution[0,i] += margin
			if polygon[1,i] < np.average(polygon[1,:]):
				solution[1,i] -= margin 
			else:
				solution[1,i] += margin
		return solution


	def toPolygonStamped(self, poly):
		p = PolygonStamped()
		p.header.frame_id = 'odom'
		p.polygon.points = [Point32() for i in range(len(poly[0]))]
		for i in range(len(p.polygon.points)):
			p.polygon.points[i].x = poly[0][i]
			p.polygon.points[i].y = poly[1][i]
		return p


if __name__ == '__main__':
	try:
		sim()
	except rospy.ROSInterruptException:
		pass
