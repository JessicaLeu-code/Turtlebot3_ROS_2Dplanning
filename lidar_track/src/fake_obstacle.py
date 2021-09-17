#!/usr/bin/env python
import rospy
from research.msg import Polygons
from geometry_msgs.msg import Pose, PoseArray, PolygonStamped, Point32

def create_obstacle(vertices):
	obstacle = PolygonStamped()
	obstacle.header.frame_id = 'odom'
	obstacle.polygon.points = [Point32() for i in range(len(vertices[0]))]
	for i in range(len(obstacle.polygon.points)):
		obstacle.polygon.points[i].x = vertices[0][i]
		obstacle.polygon.points[i].y = vertices[1][i]

	return obstacle

def create_obstacle2(vertices):
	obstacle = PolygonStamped()
	obstacle.header.frame_id = 'odom'
	obstacle.polygon.points = [Point32() for i in range(len(vertices[0]))]
	for i in range(len(obstacle.polygon.points)):
		obstacle.polygon.points[i].x = vertices[0][i]
		obstacle.polygon.points[i].y = vertices[1][i]

	return [obstacle]



def publish_obstacle():
	rospy.init_node('obstacle1')	
	obstacle_pub = rospy.Publisher('obstacle1_poly', PolygonStamped, queue_size=1)
	obstacle_pub2 = rospy.Publisher('obstacles', Polygons, queue_size=100)

	# vertices = [[2.3, 4.0, 5.2, 1.2], [0.4, 0.4, -0.5, -0.5]]
	vertices = [[1.6, 3.1, 3.4, 1.2], [0.3, 0.3, -0.5, -0.5]]
	obstacle = create_obstacle(vertices)
	obstacle2 = create_obstacle2(vertices)


	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		obstacle_pub.publish(obstacle)
		obstacle_pub2.publish(obstacle2)
		rate.sleep()

if __name__ == '__main__':
    try:
		publish_obstacle()
    except rospy.ROSInterruptException:
		pass
