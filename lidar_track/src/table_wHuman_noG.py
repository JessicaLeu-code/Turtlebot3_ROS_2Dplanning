#!/usr/bin/env python
import rospy
import numpy as np
from research.msg import Polygons
from geometry_msgs.msg import Twist, PolygonStamped, Point32

def sim():
	rospy.init_node('obstacle')	
	obstacle_poly_pub = rospy.Publisher('obstacle1_poly', PolygonStamped, queue_size=1)
	obstacle_v_pub = rospy.Publisher('obstacle1_v', Twist, queue_size=1)
	# obstacle_rviz_pub = rospy.Publisher('obstacles', Polygons, queue_size=100)
	human_pub = rospy.Publisher('/human', Point32, queue_size=10)
	human_obs_pub = rospy.Publisher('obstacle2_poly', PolygonStamped, queue_size=1)
	human_v_pub = rospy.Publisher('obstacle2_v', Twist, queue_size=1)

	vertices = [[1, 1, -1, -1], [5, 2, 2, 5]]
	vel = [[0, 0]]
	obstacle = Obstacle(vertices, vel)
	rate = rospy.Rate(10)

	human = Obstacle()

	while not rospy.is_shutdown():
		obstacle_poly_pub.publish(obstacle.toPolygonStamped())
		obstacle_v_pub.publish(obstacle.toTwist())
		# obstacle_rviz_pub.publish([obstacle.toPolygonStamped()])
		human_obs_pub.publish(human.toPolygonStamped())
		human_v_pub.publish(human.toTwist())
		human_pub.publish(human.center())
		human.step()
		rate.sleep()

class Obstacle:
	def __init__(self, vertices=[[0, 0, 1, 1], [0, 1, 1, 0]], vel = [[0,0]]):
		self.poly = np.array(vertices) * 1.0
		self.v = np.array(vel).T * 1.0
		self.dt = 0.1
		self.i = 0

	def center(self):
		cen = np.mean(self.poly, axis=1)
		cen = np.squeeze(cen)
		pt = Point32()
		pt.x = cen[0]
		pt.y = cen[1]
		return pt

	def step(self):
		if self.i == 0:
			self.v = np.array([[0], [0.4]])
		if self.i % 50 == 0:
			self.v = -1*self.v
	
		self.poly += self.v * self.dt
		self.i += 1 

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

if __name__ == '__main__':
    try:
		sim()
    except rospy.ROSInterruptException:
		pass
