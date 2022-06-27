#!/usr/bin/env python
import rospy
import math
import numpy as np
import pyclipper
from geometry_msgs.msg import Twist, PolygonStamped
from tf.transformations import euler_from_quaternion
import matplotlib.path as mpltPath
from geometry_msgs.msg import PoseStamped, Point, Point32,PointStamped
from nav_msgs.msg import Path
from lqr_module import Lqr
from cfs_module import Cfs
import time



class Follower:
	def __init__(self):
		self.waypoints = [] 			# dim x nstep array of waypoints
		self.x = [0, 0, 0, 0, 0]		# [x y z v w]
		self.i = 0  					# index of waypoints to use
		self.j = 0						# absolute index
		self.obstacle_poly = []
		self.obstacle_v = []
		self.obstacles = []
		self.obstacles2 = []
		self.lidar_obs = []
		self.lqr = Lqr()
		self.waypoints2 = []
		self.xyz = np.zeros(3)
		self.prevLalpha = 0
		self.prevLs = [0, 0, 0]
		self.goal = [2, -4]
		self.dt = 0.5
		self.mode = 1 # use feedback
		self.avg = 1 # use averaging
		self.lookahead = 4
		self.backwards = False
		self.human = [0,0,0,0]
		self.lastwaypoint = []
		self.vels = []
		self.lidar_vels = []
		self.hobs = [[],[]]
		self.pobs = []              #
		self.refpath2 =[]
		self.follower_Path = []
		self.f_waypoints = []
		self.nstep = 21
		self.f_nstep = 6
		self.meet_point = 0
		self.tra =[]
		self.total_dis = 0
		self.mindis = 100
		self.nX = 0
		self.nY = 0

		self.margin = 0
		self.gamma = 0.7            # discount constant
		self.p_stop = 0.5
		self.p_go = 0.5
		self.p_mat = []
		self.tempnsw =1
		self.count =0

	def path_callback(self, msg):
		# if self.i != 0 and len(msg.poses) == 0:
		# 	return
		self.waypoints = np.zeros([2,len(msg.poses)])

		for i in range(0, len(msg.poses)):
			self.waypoints[0,i] = msg.poses[i].pose.position.x
			self.waypoints[1,i] = msg.poses[i].pose.position.y

		# if len(msg.poses) > 0:
		# 	print("ROBOT POS", self.x[0:2])
		# 	print("PATH START", self.waypoints[:, 0])

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

		self.g_vel = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]
		#print(math.degrees(self.x[2]))


	def dt_callback(self, msg):
		self.dt = msg.data		

	def filter_callback(self,msg):
		self.xyz[0] = msg.x
		self.xyz[1] = msg.y
		self.xyz[2] = msg.z

	def goal_callback(self, msg):
		self.goal[0] = msg.x
		self.goal[1] = msg.y

	def obstacle_poly_callback(self, msg):
		pts = [[], []]
		for i in range(len(msg.polygon.points)):
			pts = np.concatenate((pts, [[msg.polygon.points[i].x], [msg.polygon.points[i].y]]), axis=1)
		self.obstacles = [pts]

		self.hobs = np.concatenate((self.hobs, np.mean(pts, axis=1).reshape(2,1)), axis=1)
		self.pobs = self.gamma*np.concatenate((self.pobs, [1]))
		self.pobs[-1] = 1
		self.pobs = self.pobs/sum(self.pobs)

	def obstacle_v_callback(self, msg):

		self.obstacle_v = np.array([[msg.linear.x, msg.linear.y]]).T

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

	def human_callback(self, msg):
		self.human[0] = msg.x
		self.human[1] = msg.y

	def human_Twist_callback(self,msg):
		self.human[2] = msg.linear.x
		self.human[3] = msg.linear.y
		#print(self.human[2])

	def scanpoints_callback(self, msg):
		n = len(msg.points)
		self.p_mat = np.zeros([2,n])
		for i in range(n):
			self.p_mat[0, i] = msg.points[i].x
			self.p_mat[1, i] = msg.points[i].y

	def tr_callback(self, msg):
		self.tra = np.zeros([2,len(msg.poses)])

		for i in range(0, len(msg.poses)):
			self.tra[0,i] = msg.poses[i].pose.position.x
			self.tra[1,i] = msg.poses[i].pose.position.y

	def move_collision_detection(self):

		if(self.waypoints == []):
			return

		if len(self.waypoints[0]) == 0:
			return Twist()

		usecfs = False
		if(self.will_collide() == True):

			obs_center = [np.average(self.obstacle_poly[0,:]), np.average(self.obstacle_poly[1,:])]

			if(obs_center[0] > self.x[0]):
				#print('safety controller')
				if usecfs is True: # not working
					self.waypoints = self.cfs.get_np([self.obstacle_poly], self.x)
				else:
					dx = obs_center[0] - self.x 
					dy = obs_center[1] - self.x[1]
					for i in range(0, 16-self.i): #min(20, self.i+16)):
						self.waypoints[0, self.i+i] = self.waypoints[0, self.i] + dy*0.1*i
						self.waypoints[1, self.i+i] = self.waypoints[1, self.i] - dx*0.1*i

		#self.erase_passed_line()
		return self.motion_control_line()

	def move(self):
		if(self.waypoints == []):
			return

		return self.motion_control_line()

	def lqr_follower(self):
		xout = self.lqr.get_lqr_reference(self.waypoints, self.x)
		self.waypoints2 = xout[0:2, :]

		cmv = Twist()
		cmv.linear.x = xout[3,1]
		cmv.angular.z = xout[4,1]
		return cmv

	def straight_or_turn(self):
		# goes straight if next waypoint is straight ahead, stops and turns otherwise
		cmv = Twist()
		des_t = math.atan(self.waypoints[1,1]-self.x[1]/(self.waypoints[0,1]-self.x[0]))
		if(abs(des_t - self.x[2]) > 0.01):
			cmv.linear.x = 0
			cmv.angular.z = 0.05*np.sign(des_t - self.x[2])
		else:
			cmv.linear.x = 0.1
			cmv.angular.z = 0
		return cmv

	'''
	def motion_control_line(self):
		# from http://kyb.fei.tuke.sk/laboratoria/ludia/pdf/Kopcik_Jadlovsky_K&I_2015.pdf

		cmv = Twist()
		#self.margin = self.get_margin_polygon()
		self.Optimize_follower_waypoints()



		self.f_waypoints = np.zeros([2,len(self.refpath2)/2])

		for m in range(len(self.refpath2)/2):
			self.f_waypoints[0,m] = self.refpath2[2*m]
			self.f_waypoints[1,m] = self.refpath2[2*m+1]

		if len(self.f_waypoints[0]) == 0:
			return cmv

		if len(self.waypoints[0]) == 0:
			return cmv

		# do a search for closest pt in trajectory
		minD = float('inf')
		for i in range(len(self.f_waypoints[0])):
			temp = np.linalg.norm(np.array(self.x[0:2]) - np.array(self.f_waypoints[:, i]))
			if temp < minD:
				minD = temp
				self.f_i = i

		minD = float('inf')
		for i in range(len(self.waypoints[0])):
			temp = np.linalg.norm(np.array(self.x[0:2]) - np.array(self.waypoints[:, i]))
			if temp < minD:
				minD = temp
				self.i = i



		#for i in range(len(self.waypoints[0])):
		#	temp = np.linalg.norm(np.array(self.x[0:2]) - np.array(self.waypoints[:, i]))
		#	if temp < minD:
		#		minD = temp
		#		self.i = i

		dy = self.x[1] - self.waypoints[1,i]
		dx = self.x[0] - self.waypoints[0,i]

		# calc top speed
		pathLen = 0.0
		diff = np.diff(self.waypoints)
		diff = np.diff(self.f_waypoints)
		for i in range(len(diff[0])):
			pathLen += np.linalg.norm(diff[:,i])

		#totalTime = self.dt*(len(self.waypoints[0])-1)
		#avgSpd = pathLen/totalTime
		# Note: minimum trajectory size of 2
		#if len(self.waypoints[0]) < 4:
		if len(self.f_waypoints[0]) < 4:
			n = 1
		else:
			n = self.lookahead # how far ahead to look, should be >= 2



		max_ind = len(self.waypoints[0]) - 1
		max_ind_ang = len(self.f_waypoints[0]) - 1

		#self.goal = self.waypoints[0:2, -1]
		#self.goal = self.f_waypoints[0:2, -1]



		if np.linalg.norm(np.array(self.x[0:2]) - np.array(self.goal)) < 0.07:
			#print("Goal Reached. Stopping...")
			cmv.linear.x = 0
			cmv.angular.z = 0

			return cmv

		if self.avg:
			nextX_ang = 0
			nextY_ang = 0
			if n > 1:
				nextX = 0
				nextY = 0
				la = [-n+1, 0, n]
				for j in la:
					nextX += self.waypoints[0, min(self.i + n + j, max_ind)]
					nextY += self.waypoints[1, min(self.i + n + j, max_ind)]
					nextX_ang += self.f_waypoints[0, min(self.f_i + n + j, max_ind_ang)]
					nextY_ang += self.f_waypoints[1, min(self.f_i + n + j, max_ind_ang)]
				nextX /= 3.0
				nextY /= 3.0
				nextX_ang /= 3.0
				nextY_ang /= 3.0
			else:
				nextX = self.waypoints[0,min(self.i+n, max_ind)]
				nextY = self.waypoints[1,min(self.i+n, max_ind)]
			nextX_ang = self.f_waypoints[0,min(self.f_i+1, max_ind_ang)]
			nextY_ang = self.f_waypoints[1,min(self.f_i+1, max_ind_ang)]
		else:
			nextX = self.waypoints[0,min(self.i+n, max_ind)]
			nextY = self.waypoints[1,min(self.i+n, max_ind)]
			nextX_ang = self.f_waypoints[0,min(self.f_i+1, max_ind_ang)]
			nextY_ang = self.f_waypoints[1,min(self.f_i+1, max_ind_ang)]

		#print("ACTUAL NEXT PT", self.waypoints[0:2, min(self.i + n, max_ind)])
		#print("AVERAGE PT", np.array([nextX, nextY]))

		if self.mode:
			La = self.x[1] - nextY
			Lb = self.x[0] - nextX
			Lc = self.x[1] * nextX - self.x[0]*nextY
			La_ang = self.x[1] - nextY_ang
			Lb_ang = self.x[0] - nextX_ang
			Lc_ang = self.x[1] * nextX_ang - self.x[0]*nextY_ang
		else:
			La = self.waypoints[1,min(self.i, max_ind)] - nextY
			Lb = self.waypoints[0,min(self.i, max_ind)] - nextX
			Lc = self.waypoints[1,min(self.i, max_ind)] * nextX - self.waypoints[0,min(self.i, max_ind)] * nextY
			La_ang = self.f_waypoints[1,min(self.f_i, max_ind)] - nextY_ang
			Lb_ang = self.f_waypoints[0,min(self.f_i, max_ind)] - nextX_ang
			Lc_ang = self.f_waypoints[1,min(self.f_i, max_ind_ang)] * nextX_ang - self.f_waypoints[0,min(self.f_i, max_ind_ang)] * nextY_ang

		#s = abs(La*self.x[0] - Lb*self.x[1] + Lc)/(np.sqrt(pow(La, 2) + pow(Lb, 2))) # distance from line
		s = abs(La_ang*self.x[0] - Lb_ang*self.x[1] + Lc_ang)/(np.sqrt(pow(La_ang, 2) + pow(Lb_ang, 2))) # distance from line

		if self.mode:
			#dy = self.x[1] - nextY
			#dx = self.x[0] - nextX
			dy = self.x[1] - nextY_ang
			dx = self.x[0] - nextX_ang
		else:
			#dy = self.waypoints[1,min(self.i, max_ind)] - nextY
			#dx = self.waypoints[0,min(self.i, max_ind)] - nextX
			dy = self.f_waypoints[1,min(self.f_i, max_ind_ang)] - nextY_ang
			dx = self.f_waypoints[0,min(self.f_i, max_ind_ang)] - nextX_ang
		cmv.linear.x = np.sqrt(pow(dx, 2) + pow(dy, 2)) / self.dt*1.2  # *1.2 is tuning parameter for real robot

		#print("raw spd",cmv.linear.x)


		self.prevLs[0] = La
		self.prevLs[1] = Lb
		self.prevLs[2] = Lc

		cmv.linear.x = min(cmv.linear.x, 0.25*np.sign(cmv.linear.x), key=abs)    #find max velocity
		# cmv.linear.x = min(cmv.linear.x, avgSpd*np.sign(cmv.linear.x), key=abs)    #find max velocity
		# print("speed:", cmv.linear.x)
		# print("avgSpd:", avgSpd)


		if abs(Lb_ang) < 0.01 and abs(La_ang) < 0.01:
			Lalpha = self.prevLalpha
		else:
			Lalpha = math.atan2(-La_ang,-Lb_ang) # angle of line in proper quadrant

		s_ang = abs(La_ang*self.x[0] - Lb_ang*self.x[1] + Lc_ang)/(np.sqrt(pow(La_ang, 2) + pow(Lb_ang, 2))) # distance from line

		if(np.isnan(s_ang)):
			s_ang = 0
		if(np.isnan(Lalpha)):
			Lalpha = 2*np.pi


		b = Lalpha - self.x[2]

		if b > np.pi:
			b -= 2*np.pi
		elif b < -np.pi:
			b += 2*np.pi

		k1 = 1
		k2 = 1.5 # can tune
		self.prevLalpha = Lalpha

		self.backwards = False
		if abs(b) > np.pi / 2:
			cmv.linear.x *= -1
			compl = min(abs(np.pi - b), abs(-np.pi - b))
			b = -1*np.sign(b)*compl
			self.backwards = True

		# print("reverse b:", b )

		cmv.angular.z = k2*(k1*b - (1-k1)*np.cos(b)*s)
		cmv.angular.z = min(cmv.angular.z, np.sign(cmv.angular.z) * 2, key=abs)

		if(cmv.linear.x<0):
			cmv.linear.x = -0.25
		else:
			cmv.linear.x = 0.25

		# time to track
		if(np.mod(self.j+1, 4) == 0):
			self.f_i += 1
			self.j += 1


		if(self.count == 0):
			if(self.waypoints == []):
				cmv.linear.x = 0
			else:
				self.count = 1

		if(self.count ==1):
			self.start_time = rospy.get_time()
			print("start")
			self.count = 2

		if(self.count == 2):
			if(self.human !=[] and self.x != []):
				dis = np.sqrt(pow(self.human[0]-self.x[0], 2) + pow(self.human[1]-self.x[1], 2))
				if(dis < self.mindis):
					self.mindis = dis
			if np.sqrt(pow((self.goal[0] - self.x[0]), 2) + pow((self.goal[1]-self.x[1]), 2)) < 0.18:
				print(np.sqrt(pow((self.goal[0]- self.x[0]), 2) + pow((self.goal[1]-self.x[1]), 2)))
				self.count = 3

		if(self.count == 3):
			elapsed_time = rospy.get_time() - self.start_time
			print("goal")
			print(elapsed_time)
			pos = np.array([[self.x[0]],[self.x[1]]])
			self.tra = np.concatenate([self.tra,pos], axis=1)

			for i in range(len(self.tra[0])-1):
				dis = np.sqrt(pow(self.tra[0,i+1]-self.tra[0,i], 2) + pow(self.tra[0,i+1]-self.tra[0,i], 2))
				self.total_dis = dis+self.total_dis
			#self.total_dis = self.total_dis + np.sqrt(pow((self.goal[0]- self.x[0]), 2) + pow((self.goal[1]-self.x[1]), 2))
			print(self.total_dis)
			print(self.mindis)
			print(self.tra)
			self.count = 4

		self.nX = nextX_ang
		self.nY = nextY_ang

		return cmv

	
	'''
	#for back up
	def motion_control_line(self):

		# from http://kyb.fei.tuke.sk/laboratoria/ludia/pdf/Kopcik_Jadlovsky_K&I_2015.pdf
		cmv = Twist()

		if len(self.waypoints[0]) == 0:
			return cmv
		
		# do a search for closest pt in trajectory
		minD = float('inf')
		for i in range(len(self.waypoints[0])):
			temp = np.linalg.norm(np.array(self.x[0:2]) - np.array(self.waypoints[:, i]))
			if temp < minD:
				minD = temp
				self.i = i

		#print(self.follower_waypoints)

		#self.margin = o.get_margin_polygon()

		dy = self.x[1] - self.waypoints[1,i]
		dx = self.x[0] - self.waypoints[0,i]

		# calc top speed
		pathLen = 0.0
		diff = np.diff(self.waypoints)
		for i in range(len(diff[0])):
			pathLen += np.linalg.norm(diff[:,i])

		totalTime = self.dt*(len(self.waypoints[0])-1)
		avgSpd = pathLen/totalTime
		# Note: minimum trajectory size of 2
		if len(self.waypoints[0]) < 4:
			n = 1

		else:
			n = self.lookahead # how far ahead to look, should be >= 2

		max_ind = len(self.waypoints[0]) - 1
		
		#self.goal = self.waypoints[0:2, -1]

		if np.linalg.norm(np.array(self.x[0:2]) - np.array(self.goal)) < 0.07:
			#print("Goal Reached. Stopping...")
			cmv.linear.x = 0
			cmv.angular.z = 0
			return cmv

		if self.avg:
			if n > 1:
				nextX = 0
				nextY = 0
				la = [-n+1, 0, n]
				for j in la:
					nextX += self.waypoints[0, min(self.i + n + j, max_ind)]
					nextY += self.waypoints[1, min(self.i + n + j, max_ind)]
				nextX /= 3.0
				nextY /= 3.0
			else:
				nextX = self.waypoints[0,min(self.i+n, max_ind)]
				nextY = self.waypoints[1,min(self.i+n, max_ind)]
		else:
			nextX = self.waypoints[0,min(self.i+n, max_ind)]
			nextY = self.waypoints[1,min(self.i+n, max_ind)]

		#print("ACTUAL NEXT PT", self.waypoints[0:2, min(self.i + n, max_ind)])
		#print("AVERAGE PT", np.array([nextX, nextY]))

		if self.mode:
			La = self.x[1] - nextY
			Lb = self.x[0] - nextX
			Lc = self.x[1] * nextX - self.x[0]*nextY
		else:
			La = self.waypoints[1,min(self.i, max_ind)] - nextY
			Lb = self.waypoints[0,min(self.i, max_ind)] - nextX
			Lc = self.waypoints[1,min(self.i, max_ind)] * nextX - self.waypoints[0,min(self.i, max_ind)] * nextY

		if abs(Lb) < 0.01 and abs(La) < 0.01:
			Lalpha = self.prevLalpha
		else:
			Lalpha = math.atan2(-La,-Lb) # angle of line in proper quadrant

		s = abs(La*self.x[0] - Lb*self.x[1] + Lc)/(np.sqrt(pow(La, 2) + pow(Lb, 2))) # distance from line

		if(np.isnan(s)):
			s = 0
		if(np.isnan(Lalpha)):
			Lalpha = 2*np.pi
		
		b = Lalpha - self.x[2]

		if b > np.pi:
			b -= 2*np.pi
		elif b < -np.pi:
			b += 2*np.pi

		k1 = 1
		k2 = 1.5 # can tune

		if self.mode:
			dy = self.x[1] - nextY
			dx = self.x[0] - nextX
		else:
			dy = self.waypoints[1,min(self.i, max_ind)] - nextY
			dx = self.waypoints[0,min(self.i, max_ind)] - nextX
		cmv.linear.x = np.sqrt(pow(dx, 2) + pow(dy, 2)) / self.dt * 0.4 # *1.2 is tuning parameter for real robot

		#print("raw spd",cmv.linear.x)

		self.prevLalpha = Lalpha
		self.prevLs[0] = La
		self.prevLs[1] = Lb
		self.prevLs[2] = Lc

		cmv.linear.x = min(cmv.linear.x, 0.25*np.sign(cmv.linear.x), key=abs)    #find max velocity
		# cmv.linear.x = min(cmv.linear.x, avgSpd*np.sign(cmv.linear.x), key=abs)    #find max velocity
		# print("speed:", cmv.linear.x)
		# print("avgSpd:", avgSpd)
		
		self.backwards = False
		if abs(b) > np.pi / 2:
			cmv.linear.x *= -1	
			compl = min(abs(np.pi - b), abs(-np.pi - b))
			b = -1*np.sign(b)*compl
			self.backwards = True

		if(cmv.linear.x<0):
			cmv.linear.x = -0.25
		else:
			cmv.linear.x = 0.25

		# print("reverse b:", b )

		cmv.angular.z = k2*(k1*b - (1-k1)*np.cos(b)*s)
		cmv.angular.z = min(cmv.angular.z, np.sign(cmv.angular.z) * 2, key=abs)

		# time to track
		if(np.mod(self.j+1, 4) == 0):
			self.i += 1
		self.j += 1


		if(self.count == 0):
			if(self.waypoints == []):
				cmv.linear.x = 0
			else:
				self.count = 1

		if(self.count ==1):
			self.start_time = rospy.get_time()
			print("start")
			self.count = 2

		if(self.count == 2):
			if(self.human !=[] and self.x != []):
				dis = np.sqrt(pow(self.human[0]-self.x[0], 2) + pow(self.human[1]-self.x[1], 2))
				if(dis < self.mindis):
					self.mindis = dis
			if np.sqrt(pow((self.goal[0] - self.x[0]), 2) + pow((self.goal[1]-self.x[1]), 2)) < 0.18:
				print(np.sqrt(pow((self.goal[0]- self.x[0]), 2) + pow((self.goal[1]-self.x[1]), 2)))
				self.count = 3

		if(self.count == 3):
			elapsed_time = rospy.get_time() - self.start_time
			print("goal")
			print(elapsed_time)
			pos = np.array([[self.x[0]],[self.x[1]]])
			self.tra = np.concatenate([self.tra,pos], axis=1)

			for i in range(len(self.tra[0])-1):
				dis = np.sqrt(pow(self.tra[0,i+1]-self.tra[0,i], 2) + pow(self.tra[0,i+1]-self.tra[0,i], 2))
				self.total_dis = dis+self.total_dis
			#self.total_dis = self.total_dis + np.sqrt(pow((self.goal[0]- self.x[0]), 2) + pow((self.goal[1]-self.x[1]), 2))
			print(self.total_dis)
			print(self.mindis)
			print(self.tra)
			self.count = 4

		self.nX = nextX
		self.nY = nextY

		#http
		return cmv
		

	def Optimize_follower_waypoints(self):

		c = Cfs(self.f_nstep, 0)
		#o = Optimizer()
		self.move_pred()
		c.set_margin(self.margin)
		self.obstacles2 = self.obstacles + self.lidar_obs
		self.goal_waypoint = np.zeros([1,2])
		self.meet_point = self.i + self.f_nstep
		self.nstep = len(self.waypoints[0])
		dis = np.sqrt(pow((self.waypoints[0,self.nstep-1]-self.waypoints[0,self.i]), 2) + pow((self.waypoints[0,self.nstep-1]-self.waypoints[0,self.i]), 2))

		if(self.nstep > self.meet_point ) :
			self.goal_waypoint = [self.waypoints[0,self.meet_point] , self.waypoints[1,self.meet_point]]

		else:
			self.goal_waypoint = [self.waypoints[0,self.nstep-1] , self.waypoints[1,self.nstep-1]]

		#self.goal_waypoint = [self.waypoints[0,len(self.waypoints[0])-1] , self.waypoints[1,len(self.waypoints[0])-1]]
		self.vels = self.vels + self.lidar_vels
		if(np.sum(self.human) != 0 ):
			self.refpath2 = c.optimize2(self.obstacles2, self.vels, self.x, self.goal, self.goal_waypoint, human=self.human)
		else:
			self.refpath2 = [0]

		#self.follower_waypoints = self.to_follower_path(self.refpath2)

		try:
			barrier = rospy.get_param('~barrier_radius')
		except:
			barrier = 1.5
		c.set_barrier(barrier)

		return self.refpath2

	def unicycle_model(self):
		# using formula xdot = v * cos(theta), ydot = v * sin(theta), thetadot = omega
		xdot = self.waypoints[0,2] - self.x[0]
		ydot = self.waypoints[1,2] - self.x[1] 
		thetadot = math.atan(ydot/xdot)

		cmv = Twist()
		cmv.linear.x = np.mean([xdot/(np.cos(self.x[2])+0.0001), abs(ydot/np.sin(self.x[2])+0.0001)])
		if np.isnan(cmv.linear.x):
			cmv.linear.x = 0.1
		elif cmv.linear.x > 0.5:	
			cmv.linear.x = 0.5
		elif cmv.linear.x < -0.5:
			cmv.linear.x = -0.5

		cmv.angular.z = thetadot
		if cmv.angular.z > 0.5:
			cmv.angular.z = 0.5
		elif cmv.angular.z < -0.5:
			cmv.angular.z = -0.5

		cmv.linear.x = np.mean([cmv.linear.x, self.x[3]])
		cmv.angular.z = np.mean([cmv.angular.z, self.x[4]])
		return cmv

	def will_collide(self):
		will_collide = False

		if len(self.waypoints) == 0 or len(self.obstacle_poly) == 0:
			return will_collide

		for i in range(0, 10):
			turtle_np = self.waypoints[:,i] # np array turtle location at dt*i
			turtle = [(turtle_np[0], turtle_np[1])]

			obstacle_np = self.obstacle_poly + self.obstacle_v*i*0.5 # np array obstacle vertices at dt*i
			obstacle_margin = self.offset_polygon(obstacle_np, 20)
			obstacle_mplt = mpltPath.Path(obstacle_margin)
			inside = obstacle_mplt.contains_points(turtle)

			if inside == True:
				will_collide = True 
				break

		return will_collide

	def toPath(self, msg):
		if len(msg) == 0:
			return
		poseArray = []
		for i in range(len(msg[0,:])):
			poses = PoseStamped()
			poses.pose.position.x = msg[0,i]
			poses.pose.position.y = msg[1,i]
			poseArray += [poses]
		waypoints = Path()
		waypoints.header.frame_id = 'odom'
		waypoints.poses = poseArray
		return waypoints
	'''
	def toTrajectory(self ):

		dis = np.sqrt(pow(self.old_x-self.x[0], 2) + pow(self.old_y-self.x[1], 2))
		if dis > 0.05 :
			self.tra_x.append(self.x[0])
			self.tra_y.append(self.x[1])
			self.old_x = self.x[0]
			self.old_y = self.x[1]
			self.tra_num = self.tra_num + 1
			self.total_dis = self.total_dis + dis
			#print(dis)
			#if(self.count <3):
				#print(-1*self.x[1],self.x[0])


		poseArray = []

		if self.tra_num == 0:
			return

		for i in range(self.tra_num):
			poses = PoseStamped()
			poses.pose.position.x = self.tra_x[i]
			poses.pose.position.y = self.tra_y[i]
			poseArray += [poses]
		trajectory = Path()
		trajectory.header.frame_id = 'odom'
		trajectory.poses = poseArray

		return trajectory
	'''

	# input: np array
	# output: list of [x,y]
	def offset_polygon(self, polygon, margin):

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

	def erase_passed_line(self):
		self.m = 0
		self.Trigger_find_firstwaypoint = -1

		if(self.x[3]<0):
			self.goingback =180
		else:
			self.goingback =0

		self.rad = np.deg2rad(90 - math.degrees(self.x[2]) +self.goingback)
		self.rot = np.array([[np.cos(self.rad), -np.sin(self.rad)], [np.sin(self.rad), np.cos(self.rad)]])

		if(self.waypoints == []):

			return

		else:

			for i in range(len(self.waypoints[0])):

				self.Newwaypoints = np.array([self.waypoints[0,i] - self.x[0], self.waypoints[1,i] - self.x[1]])
				self.rotated = np.dot(self.rot, self.Newwaypoints)

				if(self.rotated[1] > 0 and self.Trigger_find_firstwaypoint < 0 and i < len(self.waypoints[0])) :
					self.Trigger_find_firstwaypoint = i


				if(self.Trigger_find_firstwaypoint > 0):
					self.waypoints[0,self.m] = self.waypoints[0,i-1]
					self.waypoints[1,self.m] = self.waypoints[1,i-1]
					self.m = self.m + 1

	def to_follower_path(self, refpath):
		poseArray = []

		for i in range(len(refpath)/2):
			poses = PoseStamped()
			poses.pose.position.x = refpath[2*i]
			poses.pose.position.y = refpath[2*i+1]
			poseArray += [poses]

		follower_path = Path()
		follower_path .header.frame_id = 'odom'
		follower_path .poses = poseArray
		return follower_path


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
		return 0.3+omega[1]

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

	def measure_min_dis(self):
		if(self.human  == []):
			x1 = 0
			y1 = 0
		else:
			#obs = self.obstacles[0]
			#x1 = 0
			#y1 = 0
			x1 = self.nX
			y1 = self.nY

		min_point = PointStamped()
		min_point.header.frame_id = 'odom'

		min_point.point.x = x1
		min_point.point.y = y1
		'''
		min_dis = 100
		
		if(self.lidar_obs == []):
			return
		else:
			for i in range(len(self.lidar_obs)):
				obs = self.lidar_obs[i]

				for j in range(len(obs[0])):
					x2 = obs[0,j]
					y2 = obs[1,j]
					dx = x1-x2
					dy = y1-y2
					dis = np.sqrt(pow(dx, 2) + pow(dy, 2))

					if(dis < min_dis):
						min_dis = dis
						min_point.point.x = x2
						min_point.point.y = y2
			#print(x2)
			#print(y2)
		'''

		return min_point
