import numpy as np

from util import *
from viz_obs import Viz

from pykalman import KalmanFilter

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped, Point32, Point, Twist
from lidar_track.msg import Polygons
from nav_msgs.msg import Path
from std_msgs.msg import UInt16
from tf.transformations import euler_from_quaternion
from tf import TransformListener
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from multiprocessing import Queue

import sys

import rospy

# makes polygons from lines by creating squres with sides equal to the line length
def make_polys(lines, tf):
	obs = [PolygonStamped() for _ in range(len(lines))]
	obs_centers_areas = list()
	lines = parallel(lines, 0.3)

	for i, line in enumerate(lines):
		poly = obs[i].polygon
		poly.points = [Point32() for _ in range(4)]

		# we need to make sure that we "extend" the square out in the correct
		# direction. first, we place the opposite edge of the square:
		line_len = norm(line)
		

		line1 = shift(line, line_len, dir=1)
		line2 = shift(line, line_len, dir=-1)
		if dist_to_robot(line1, tf) > dist_to_robot(line2, tf):
			opp_line = line1
		else:
			opp_line = line2


		# now we know the four points, so we can finish this polygon:
		p0, p1 = line
		p2, p3 = opp_line

		obs_x = 0
		obs_y = 0

		for j, pt in enumerate((p0, p1, p3, p2)):
			ppt = poly.points[j]
			x,y = pt
			ppt.x = x
			ppt.y = y

		obst = Obstacle(obs[i])

		#check if similar to previous objects
		flag = False
		for data in obs_centers_areas:
			flag = obst.sim_check(data)

		if not flag:
			obs_centers_areas.append(obst)

	ret = Polygons()
	ret.polygons = obs

	#print("NUMBER OF LINES DETECTED", len(obs_centers_areas))

	return ret, obs_centers_areas

# contains PolygonStamped object, raw measurements, kalman filter, and updated estimates of the obstacle
class Obstacle(object):
	def __init__(self, polygon, kf=None):
		self.poly = polygon  #PolygonStamped
		self.center = self.center_sq() #measured center
		self.area = self.area() #meas area
		self.kf = kf
		self.mean = [self.center[0],0,0,self.center[1],0,0,self.area] #est. center
		self.cov = np.zeros((7,7))
		self.isHuman = False

	# returns midpt of the measured line from lidar
	def center(self):
		if self.poly == None:
			return

		obs_x = 0
		obs_y = 0

		for i in range(2):
			pt = self.poly.polygon.points[i]
			x = pt.x
			y = pt.y

			obs_x += x
			obs_y += y

		obs_p = [obs_x/2, obs_y/2]
		return obs_p

	def center_sq(self):
		if self.poly == None:
			return

		obs_x = 0
		obs_y = 0

		for i in range(4):
			pt = self.poly.polygon.points[i]
			x = pt.x
			y = pt.y

			obs_x += x
			obs_y += y

		obs_p = [obs_x/4, obs_y/4]
		return obs_p

	def area(self):
		p0 = self.poly.polygon.points[0]
		p1 = self.poly.polygon.points[1]
		return (np.linalg.norm(np.array([p0.x, p0.y]) - np.array([p1.x, p1.y])))**2

	# returns true if target obstacle is similar to this obstacle
	def sim_check(self, obst):
		o2 = obst.center_sq()
		o1 = self.center_sq()
		dist = np.sqrt((o2[0] - o1[0])**2 + (o2[1] - o1[1])**2)
		area_diff = np.abs(obst.area - self.area) / self.area

		if area_diff < 0.6 and dist < 0.6*np.sqrt(self.area):
			return True
		# sometimes reads bad line ontop on robot itself?
		# elif np.linalg.norm(self.center) < 0.4:
		# 	return True
		else:
			return False

	def init_kalman(self):
		dt = 0.2
		A = [[1, dt, dt**2/2, 0, 0, 0, 0],[0, 1, dt, 0, 0, 0, 0],[0, 0, 1, 0, 0, 0, 0],[0,0,0,1, dt, dt**2/2, 0],[0,0,0,0, 1, dt, 0],[0,0,0,0,0,1, 0],[0,0,0,0,0,0,1]]
		Q = np.diag([0.1, 0.01, 0, 0.1, 0.01, 0, 0.001])
		C = [[1,0,0,0,0,0,0], [0,0,0,1,0,0,0],[0,0,0,0,0,0,1]]
		R = np.diag([0.05, 0.05, 0.004])
		kf = KalmanFilter(transition_matrices=A, observation_matrices=C, transition_covariance=Q, observation_covariance=R)
		return kf

	# updates the PolygonStamped with the new polygon created from the kalman estimate
	def update_poly(self):
		length = np.sqrt(self.mean[6])
		p0 = [self.poly.polygon.points[0].x, self.poly.polygon.points[0].y]
		p1 = [self.poly.polygon.points[1].x, self.poly.polygon.points[1].y]
		line = (p0, p1)
		vec = line2vec(line)
		####################
		m1 = length/2*vec+[self.mean[0], self.mean[3]]
		m2 = -length/2*vec+[self.mean[0], self.mean[3]]
		orth = np.array([vec[1], -vec[0]])

		new_p0 = length/2*orth+m1
		new_p2 = -length/2*orth+m1
		new_p1 = length/2*orth+m2
		new_p3 = -length/2*orth+m2
		####################

		self.poly.polygon.points[0].x = new_p0[0]
		self.poly.polygon.points[0].y = new_p0[1]
		self.poly.polygon.points[1].x = new_p1[0]
		self.poly.polygon.points[1].y = new_p1[1]
		self.poly.polygon.points[3].x = new_p2[0]
		self.poly.polygon.points[3].y = new_p2[1]
		self.poly.polygon.points[2].x = new_p3[0]
		self.poly.polygon.points[2].y = new_p3[1]

	# viz of measured obs
	def get_marker(self, i):
		marker = Marker()
		marker.header.frame_id = "/odom"
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD
		marker.id = i
		marker.scale.x = 0.06
		marker.scale.y = 0.06
		marker.scale.z = 0.06
		marker.color.g = 1.0
		marker.color.a = 1.0
		pts = []
		for pt in self.poly.polygon.points:
			nw = Point()
			nw.x = pt.x
			nw.y = pt.y
			pts.append(nw)

		pts.append(pts[0])
		marker.points = pts
		return marker

	# viz for est obs
	def get_marker_meas(self, i):
		marker = Marker()
		marker.header.frame_id = "/odom"
		marker.type = marker.LINE_STRIP
		marker.action = marker.ADD
		marker.id = i+100
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.color.b = 1.0
		marker.color.a = 1.0
		pts = []
		for pt in self.poly.polygon.points:
			nw = Point()
			nw.x = pt.x
			nw.y = pt.y
			pts.append(nw)

		pts.append(pts[0])
		marker.points = pts
		return marker

	# returns velocity as Twist
	def toVel(self):
		vel = Twist()
		vel.linear.x = self.mean[1]
		vel.linear.y = self.mean[4]
		return vel

# for now, if no human exists, assume closest object to table is the human. Returns human
def checkHuman(objects, prevHuman=None):
	humanExists = False
	closest = None

	dist_mat = np.zeros(len(objects))
	area_mat = np.zeros(len(objects))

	for i,o in enumerate(objects):	
		dist_mat[i] = 0.1*((o.mean[0] - 0)**2 + (o.mean[3] - 4)**2)    # TODO: change location of table
		if not prevHuman is None:
			dist, area = calc_sim(o, prevHuman)
			dist_mat[i] += dist
			area_mat[i] = area

	
	d_max = np.amax(dist_mat) + 0.000001
	dist_mat = np.sqrt(dist_mat)
	dist_mat = dist_mat/d_max

	if not prevHuman is None:
		a_max = np.amax(area_mat)
		area_mat = area_mat/a_max

	alpha = 0.85 # TUNE THIS
	beta = 1 - alpha

	cost_mat = alpha * dist_mat + beta * area_mat
	m = np.argmin(cost_mat)
	# obstacle at index i is associated with the previous obstacle at index m[i]
	#print("human_data_asso:", m)
	closest = objects[m]

	return closest




class FindObstacles(object):
	# we increment self.id at every complete scan.

	# per complete scan, we publish (potentially) multiple PolygonStamped
	# msgs to /obstacles, all with the same header.seq value (self.id)
	def __init__(self, viz=True, experiment=None):
		self.id = 1
		rospy.init_node('obstacles_node')
		self.tf = TransformListener()
		self.num_obj = 11
		
		pub = rospy.Publisher('/obstacles', Polygons, queue_size=50)
		
		obstacle_viz = rospy.Publisher('/obstacles_viz', MarkerArray, queue_size=50)

		if experiment:
			if experiment == "-b":
				#print("INIT EXPERIMENT")
				goal_pub = rospy.Publisher('/goal_state', Point32, queue_size=10)
				human_obs_pub = rospy.Publisher('obstacle2_poly', PolygonStamped, queue_size=1)
				human_v_pub = rospy.Publisher('obstacle2_v', Twist, queue_size=1)
				self.num_obj = 1
			if experiment == '-6':
				#print("INIT EXPERIMENT")
				human_pub = rospy.Publisher('/human', Point32, queue_size=10)
				human_obs_pub = rospy.Publisher('obstacle2_poly', PolygonStamped, queue_size=1)
				human_v_pub = rospy.Publisher('obstacle2_v', Twist, queue_size=1)
				self.num_obj = 1
			if experiment == '-g':
				#print("MODE SWITCHING ENABLED")
				goal_pub = rospy.Publisher('/goal_state', Point32, queue_size=10)
				human_pub = rospy.Publisher('/human', Point32, queue_size=10)
				human_obs_pub = rospy.Publisher('obstacle2_poly', Polygons, queue_size=1)
				human_v_pub = rospy.Publisher('obstacle2_v', Twist, queue_size=1)
				self.num_obj = 3

		# keep track of real lines from the last frame
		self.prev_real_lines = set()
		# keep track of img and lines from the last frame
		self.latest_img, self.latest_lines = np.zeros((GRID, GRID)), []

		self.x = [0, 0, 0]
		self.prev = None
		self.prevHuman = None

		# MODE 0: Goal Following, 1: Assistant
		self.mode = 0

		def odom_callback(msg):
			dd,asd,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
			self.x = [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
			##print("turtlebot ODOM", self.x)

		def mode_callback(msg):
			self.mode = msg.data

		rospy.Subscriber('/odom', Odometry, odom_callback)
		rospy.Subscriber('/mode', UInt16, mode_callback)
		

		def find_lines_and_pub(lidar_data):
			#lines, img = find_lines(lidar_data, self.prev_real_lines, self.x)

			# apply transformation
			try:
				pos, qu = self.tf.lookupTransform("/odom", "/base_scan", rospy.Time())
				##print("TRANS", pos, qu)
				t_mat = self.tf.fromTranslationRotation(pos, qu)


				n = len(lidar_data.points)
				p_mat = np.ones((4, n))
				for i in range(n):
					pt = lidar_data.points[i]
					p_mat[0, i] = pt.x
					p_mat[1, i] = pt.y
					p_mat[2, i] = 0
				p_trans = np.dot(t_mat, p_mat)

			except:
				return


			# process line data into usable form
			lines = set()
			for i in range(n/2):
				line = ((p_trans[0, 2*i], p_trans[1, 2*i]), (p_trans[0, 2*i+1], p_trans[1, 2*i+1]))
				lines.add(line)


			polys, obs = make_polys(lines, pos)
			
			# associates meas obs with previous obs
			obs_fil, flag = data_association(obs, self.prev, self.num_obj)

			pts = []
			visual = MarkerArray()

			ctr = 1
			for o in obs_fil:

				# inits kf if one doesn't exist for this obs
				if o.kf == None:
					#print("INITIALIZING KF")
					o.kf = o.init_kalman()

				# update kalman
				meas = np.append(o.center, o.area)
				o.mean, o.cov = o.kf.filter_update(o.mean, o.cov, observation=meas)

				# update
				o.center = [o.mean[0], o.mean[3]]
				o.area = o.mean[6]

				# visualization
				add = Point()
				add.x = o.mean[0]
				add.y = o.mean[3]
				pts.append(add)

				visual.markers.append(o.get_marker_meas(ctr))
				o.update_poly()
				visual.markers.append(o.get_marker(ctr))
				ctr += 1

			marker = Marker()
			marker.header.frame_id = "/odom"
			marker.id = 0
			marker.type = marker.POINTS
			marker.action = marker.ADD
			marker.scale.x = 0.2
			marker.scale.y = 0.2
			marker.scale.z = 0.2
			marker.color.b = 1.0
			marker.color.a = 1.0
			marker.points = pts
			visual.markers.append(marker)
			obstacle_viz.publish(visual)

			# only allow "good" measurements to be the references
			if len(obs_fil) == self.num_obj and not flag:
				self.prev = obs_fil

			# #print("Kalman est:", self.mean)
			if experiment:
				if experiment == "-b":
					if len(obs_fil) == self.num_obj:
						o = obs_fil[0]
						cen = o.center_sq()
						goal = Point32()
						goal.x = cen[0]
						goal.y = cen[1]
						#goal_pub.publish(goal)
						#human_pub.publish(o.poly)
						#human_v_pub.publish(o.toVel())
				if experiment == "-6":
					if len(obs_fil) == self.num_obj:
						o = obs_fil[0]
						cen = o.center_sq()
						goal = Point32()
						goal.x = cen[0]
						goal.y = cen[1]
						human_pub.publish(goal)
						human_obs_pub.publish(o.poly)
						human_v_pub.publish(o.toVel())
				if experiment == "-g":
					if len(obs_fil) >= self.num_obj:       # currently hard-coded to assume num_obj objects
						polys = Polygons()
						o = checkHuman(obs_fil, prevHuman=self.prevHuman)
						self.prevHuman = o
						cen = o.center_sq()
						human_cen = Point32()
						human_cen.x = cen[0]
						human_cen.y = cen[1]
						for obj in obs_fil:
							polys.polygons.append(obj.poly)
							polys.vels.append(obj.toVel())

						if self.mode == 0:
							human_pub.publish(human_cen)
							human_obs_pub.publish(polys)
							human_v_pub.publish(o.toVel())
						elif self.mode == 1:
							goal = Point32()
							goal.x = human_cen.x
							goal.y = human_cen.y - 1.5
							goal_pub.publish(goal)
							human_pub.publish(human_cen)
							human_obs_pub.publish(polys)
							human_v_pub.publish(o.toVel())


		rospy.Subscriber('/publish_line_markers', Marker, find_lines_and_pub)
		#rospy.Subscriber('/scan', LaserScan, find_lines_and_pub)
		#rospy.Subscriber('/scan_filtered', LaserScan, find_lines_and_pub)
		rospy.spin()
