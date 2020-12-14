#!/usr/bin/env python
import rospy
import numpy as np
from research.msg import Polygons
from geometry_msgs.msg import Twist, PolygonStamped, Point32

def sim():
	rospy.init_node('obstacle')	
	obstacle_poly_pub = rospy.Publisher('obstacle1_poly', PolygonStamped, queue_size=1)
	obstacle_v_pub = rospy.Publisher('obstacle1_v', Twist, queue_size=1)
	goal_pub = rospy.Publisher('goal_state', Point32, queue_size=10)
	obstacle_rviz_pub = rospy.Publisher('obstacles', PolygonStamped, queue_size=1)

	vertices = genPoly([2,0], 4)
	vel = [[0, 0.05]]
	obstacle = Obstacle(vertices, vel)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		obstacle_poly_pub.publish(obstacle.toPolygonStamped())
		obstacle_v_pub.publish(obstacle.toTwist())
		goal_pub.publish(obstacle.toPoint())
		obstacle_rviz_pub.publish(obstacle.toPolygonStamped())
		obstacle.step()
		rate.sleep()


def genPoly(center, sides):
	length = 1
	step = 2 * np.pi / sides
	vert = np.empty((0,2))
	for i in range(sides):
		x = length * np.cos(step*i)
		y = length * np.sin(step*i)
		new_x = x + center[0]
		new_y = y + center[1]
		vert = np.vstack((vert, [new_x, new_y]))
	return vert.T


class Obstacle:
	def __init__(self, vertices=[[0, 0, 1, 1], [0, 1, 1, 0]], vel = [[0,0]]):
		self.poly = np.array(vertices) * 1.0
		self.v = np.array(vel).T * 1.0
		self.dt = 0.1
		self.i = 0
		self.center = np.mean(self.poly, axis=1) - [0, 0]

	def step(self):
		if self.i == 150:
			self.v = np.array([[0], [0.07]])
			print("start working")
		if self.i == 230:
			self.v = np.array([[0.05], [0]])
			print("start moving to right")
		if self.i == 600:
			self.v = np.array([[0], [0.05]])
			print("start moving away")
		if self.i == 900:
			self.v = np.array([[-0.05], [0]])
			print("start moving away")
		if self.i == 1200:
			self.v = np.array([[0], [-0.05]])
			print("start moving away")
		if self.i > 150 and self.i < 230:
			if np.mod(self.i, 5) == 0:
				self.v = -1*self.v
		self.poly += self.v * self.dt
		self.i += 1 
		self.center = np.mean(self.poly, axis=1) - [0, 0]

	def toPolygonStamped(self):
		p = PolygonStamped()
		p.header.frame_id = 'odom'
		p.polygon.points = [Point32() for i in range(len(self.poly[0]))]
		for i in range(len(p.polygon.points)):
			p.polygon.points[i].x = self.poly[0][i]
			p.polygon.points[i].y = self.poly[1][i]
		return p

	def toTwist(self):
		t = Twist()
		t.linear.x = self.v[0]
		t.linear.y = self.v[1]
		return t

	def toPoint(self):
		p = Point32()
		p.x = self.center[0]
		p.y = self.center[1]
		return p

if __name__ == '__main__':
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
