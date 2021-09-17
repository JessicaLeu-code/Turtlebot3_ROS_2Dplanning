#!/usr/bin/env python
import math
import numpy as np
import pyclipper
import csv
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import matplotlib.path as mpltPath
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from lqr_module import Lqr
from cfs_module import Cfs


class Follower:
	def __init__(self):
		self.waypoints = [] 			# dim x nstep array of waypoints
		self.x = [0, 0, 0, 0, 0]		# [x y z v w]
		self.i = 0  					# index of waypoints to use
		self.j = 0						# absolute index
		self.obstacle_poly = []			
		self.obstacle_v = []
		self.lqr = Lqr()
		self.waypoints2 = []
		self.cfs = Cfs(21, .6)
		self.xyz = np.zeros(3)
		self.prevLalpha = 0
		self.prevLs = [0, 0, 0]
		self.goal = None
		self.dt = 0.5

	def path_callback(self, msg):
		if self.i != 0 and len(msg.poses) == 0:
			return
		self.waypoints = np.zeros([2,len(msg.poses)])
		
		self.i = 0
		self.j = 0
		for i in range(0, len(msg.poses)):
			self.waypoints[0,i] = msg.poses[i].pose.position.x
			self.waypoints[1,i] = msg.poses[i].pose.position.y

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
		
	def dt_callback(self, msg):
		self.dt = msg.data		

	def filter_callback(self,msg):
		self.xyz[0] = msg.x
		self.xyz[1] = msg.y
		self.xyz[2] = msg.z



	def obstacle_poly_callback(self, msg):
		pts = [[], []]
		for i in range(len(msg.polygon.points)):
			pts = np.concatenate((pts, [[msg.polygon.points[i].x], [msg.polygon.points[i].y]]), axis=1)
		self.obstacle_poly = pts

	def obstacle_v_callback(self, msg):

		self.obstacle_v = np.array([[msg.linear.x, msg.linear.y]]).T

	def move_collision_detection(self):
		if(self.waypoints == []):
			return

		usecfs = True

		if(self.will_collide() == True):
			obs_center = [np.average(self.obstacle_poly[0,:]), np.average(self.obstacle_poly[1,:])]
			if(obs_center[0] > self.x[0]):
				print('safety controller')
				if usecfs is True:
					return # TODO: fix
					self.waypoints = self.cfs.get_np([self.obstacle_poly], self.x)
				else:
					dx = obs_center[0] - self.x 
					dy = obs_center[1] - self.x[1]
					for i in range(0, 16-self.i): #min(20, self.i+16)):
						self.waypoints[0, self.i+i] = self.waypoints[0, self.i] + dy*0.1*i
						self.waypoints[1, self.i+i] = self.waypoints[1, self.i] - dx*0.1*i
		# else:

			# print('not colliding')

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

	def motion_control_line(self):
		# from http://kyb.fei.tuke.sk/laboratoria/ludia/pdf/Kopcik_Jadlovsky_K&I_2015.pdf
		cmv = Twist()
		# print("dt:", self.dt)
		# cmv.linear.x = self.x[3]
		# cmv.angular.z = self.x[4]
		
		# Note: minimum trajectory size of 2
		if len(self.waypoints) < 4:
			n = 1
		else:
			n = 3 # how far ahead to look
		
		# TODO: fix cases where traj is less than 20
		max_ind = len(self.waypoints) - 1
		
		#if np.linalg.norm(np.array(self.x[0:2]) - np.array(self.goal)) < 0.05:
			#cmv.linear.x = 0
			#cmv.angular.z = 0
			#return cmv
		
		if self.i > min(30, max_ind) or self.i + n > min(30, max_ind):
			#cmv.linear.x = self.x[3]
			#cmv.angular.z = self.x[4]
			self.goal = self.waypoints[0:2, -1]
			#print("goal", self.goal)
			# stop when trajectory is too long
			if self.i + n > 1000:
				print("Trajectory exceeded limit. Stopping...")
				cmv.linear.x = 0
				cmv.angular.z = 0
				return cmv
			if np.linalg.norm(np.array(self.x[0:2]) - np.array(self.goal)) < 0.1:
				print("Goal Reached. Stopping...")
				cmv.linear.x = 0
				cmv.angular.z = 0
				return cmv

		
		# La = self.x[1] - self.waypoints[1, min(self.i+n, 20)]
		# Lb = self.x[0] - self.waypoints[0, min(self.i+n, 20)]
		# Lc = self.x[1] * self.waypoints[0, min(self.i+n, 20)] - self.x[0]
		if self.i >= max_ind:
			# La = self.prevLs[0]
			# Lb = self.prevLs[1]
			# Lc = self.prevLs[2]
			print("Traj end")
			La = self.x[1] - self.waypoints[1, min(self.i+n, max_ind)]
			Lb = self.x[0] - self.waypoints[0, min(self.i+n, max_ind)]
			Lc = self.x[1] * self.waypoints[0, min(self.i+n, max_ind)] - self.x[0]*self.waypoints[1,min(self.i+n, max_ind)]
		else:
			La = self.waypoints[1,min(self.i, max_ind)] - self.waypoints[1,min(self.i+n, max_ind)]
			Lb = self.waypoints[0,min(self.i, max_ind)] - self.waypoints[0,min(self.i+n, max_ind)]
			Lc = self.waypoints[1,min(self.i, max_ind)] * self.waypoints[0,min(self.i+n, max_ind)] - self.waypoints[0,min(self.i, max_ind)] * self.waypoints[1,min(self.i+n, max_ind)]


		if abs(Lb) < 0.01 and abs(La) < 0.01:
			Lalpha = self.prevLalpha
			# print("same")
		else:
			Lalpha = math.atan2(-La,-Lb) # angle of line in proper quadrant

		s = abs(La*self.x[0] - Lb*self.x[1] + Lc)/(np.sqrt(pow(La, 2) + pow(Lb, 2))) # distance from line
		
		if(np.isnan(s)):
			s = 0
		if(np.isnan(Lalpha)):
			Lalpha = 2*np.pi
		
		b = Lalpha - self.x[2]

		# print("b:", b )

		if b > np.pi:
			b -= 2*np.pi
		elif b < -np.pi:
			b += 2*np.pi

		# params for hardware
		# k1 = 1.2
		# k2 = 0.8
		k1 = 1.0
		k2 = 1.0
		
		# end = len(self.waypoints) - 1

		# print("b:", b)
		# print("cos(b):", abs(np.cos(b)))
		if self.i >= max_ind:
			dy = self.x[1] - self.waypoints[1,min(self.i+n, max_ind)]
			dx = self.x[0] - self.waypoints[0,min(self.i+n, max_ind)]
			cmv.linear.x = np.sqrt(pow(dx, 2) + pow(dy, 2)) * abs(np.cos(b))
		else:
			dy = self.waypoints[1,min(self.i, max_ind)] - self.waypoints[1,min(self.i+n, max_ind)]
			dx = self.waypoints[0,min(self.i, max_ind)] - self.waypoints[0,min(self.i+n, max_ind)]
			cmv.linear.x = np.sqrt(pow(dx, 2) + pow(dy, 2)) / self.dt * 1.2 # dt=0.2 from matlab traj

		#cmv.linear.x = np.sqrt(pow(dx, 2) + pow(dy, 2))*np.cos(2*b)
		#cmv.angular.z = k2*(k1*b - (1-k1)*np.cos(b)*s)*self.x[0]

		self.prevLalpha = Lalpha
		self.prevLs[0] = La
		self.prevLs[1] = Lb
		self.prevLs[2] = Lc

		cmv.linear.x = min(cmv.linear.x, 0.4*np.sign(cmv.linear.x), key=abs)    #find max velocity
		print("speed:", cmv.linear.x)

		cmv.angular.z = k2*(k1*b - (1-k1)*np.cos(b)*s)*2
		cmv.angular.z = min(cmv.angular.z, np.sign(cmv.angular.z) * 0.9, key=abs)
		print("ang speed:", cmv.angular.z)

		# time to track
		if(np.mod(self.j+1, 5) == 0):
			self.i += 1
		self.j += 1

		# print("x:", self.x[0])
		# print("y:", self.x[1])

		return cmv

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
	
	# input: np array
	# output: list of [x,y]
	def offset_polygon(self, polygon, margin):
		obstacle = ()
		for i in range(len(polygon[0])):
			obstacle = ((int(polygon[0,i]*100), int(polygon[1,i]*100)),) + obstacle
		pco = pyclipper.PyclipperOffset()
		pco.AddPath(obstacle, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
		solution = pco.Execute(margin)
		solution = np.array(solution[0])/100.
		return solution.tolist()
