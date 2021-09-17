#!/usr/bin/env python
import rospy
import rosnode
import numpy as np
from research.msg import Polygons
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, PolygonStamped, Pose
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from cfs_module import Cfs

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

def sim():
    rospy.init_node('optimization')
    o = Optimizer()
    c = Cfs(21, 0.6)

    rospy.Subscriber('/odom', Odometry, o.odom_callback)
    rospy.Subscriber('/goal', PoseStamped, o.goal_callback)
    # rospy.Subscriber('/obstacles', Polygons, o.obstacle_callback)
    rospy.Subscriber('/obstacle1_poly', PolygonStamped, o.obstacle_poly_callback)

    path_pub = rospy.Publisher('/path', Path, queue_size=100)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
    	points = c.optimize(o.obstacles, [np.zeros((2,1))], o.x, o.goal)
    	path_pub.publish(to_path(points))

        rate.sleep()

class Optimizer:    
    def __init__(self):
        self.x = [0,0,0,0,0]
        self.obstacles = []
        self.goal = [3, 0]


    def odom_callback(self, msg):
        _,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.x = [msg.pose.pose.position.x,
                  msg.pose.pose.position.y,
                  yaw,
                  msg.twist.twist.linear.x,
                  msg.twist.twist.angular.z]

    def goal_callback(self, msg):
        self.goal = [msg.pose.position.x, msg.pose.position.y]

    # def obstacle_callback(self, msg):
    #     self.obstacles += [self.polygonstampednp(p) for p in msg.polygons]
    #     num_obs = len(self.obstacles)
    #     self.obstacles = self.obstacles[(num_obs // 5):]
    #     self.obstacles = self.obstacles[:3]

    # def polygonstampednp(self, msg):
    #     arr = np.array([[],[],[]])
    #     Rz = np.array([[np.cos(-self.x[2]), -np.sin(-self.x[2]), 0], [np.sin(-self.x[2]), np.cos(-self.x[2]), 0], [0, 0, 1]])
    #     for i in range(len(msg.polygon.points)):
    #         pts_body = self.vectornp(msg.polygon.points[i])
    #         pts_earth = Rz.dot(pts_body) + np.array([[self.x[0]], [self.x[1]], [0]])
    #         arr = np.concatenate([arr, pts_earth], axis=1)
    #     return arr
    
    # def vectornp(self, msg):
    #     return np.array([[msg.x, msg.y, msg.z]]).T

    def obstacle_poly_callback(self, msg):
        pts = [[], []]
        for i in range(len(msg.polygon.points)):
            pts = np.concatenate((pts, [[msg.polygon.points[i].x], [msg.polygon.points[i].y]]), axis=1)
        self.obstacles = [pts]

if __name__ == '__main__':
    try:
        sim()
    except rospy.ROSInterruptException:
        pass
