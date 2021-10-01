#!/usr/bin/env python
import rospy 
import pyclipper
import numpy as np
from research.msg import Polygons
from geometry_msgs.msg import Twist, PoseArray, PoseStamped, PolygonStamped, Point32, Point, PointStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from cfs_module import Cfs


def sim():
    rospy.init_node('optimization')
    o = Optimizer()
    c = Cfs(21, 0)

    rospy.Subscriber('/odom', Odometry, o.odom_callback)
    rospy.Subscriber('/filteredGPS', Point, o.filter_callback)
    rospy.Subscriber('/table_poly', PolygonStamped, o.obstacle_poly_callback)
    rospy.Subscriber('/table_v', Twist, o.obstacle_vel_callback)
    rospy.Subscriber('/goal_state', Point32, o.goal_callback)

    path_pub = rospy.Publisher('/path', Path, queue_size=100)
    margin_rviz_pub = rospy.Publisher('/margins', PolygonStamped, queue_size=100)
    goal_pub = rospy.Publisher('/goalrviz', PointStamped, queue_size=100)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        o.move_pred()
        c.set_margin(o.margin)
	#o.obstacles = []
        refpath = c.optimize(o.obstacles, o.vels, o.x, o.goal)
    	path = to_path(refpath)
        margin = o.get_margin_polygon()

    	path_pub.publish(path)
        margin_rviz_pub.publish(margin)
        goal_pub.publish(o.get_goal())

        rate.sleep()


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
        self.vels = []
        self.hobs = [[],[]]
        self.pobs = []				# 
        self.gamma = 0.7 			# discount constant
        self.p_stop = 0.5
        self.p_go = 0.5
        self.xyz = np.zeros(3)
        self.goal = [0, 0]

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
        

    def filter_callback(self, msg):
        self.xyz[0] = msg.x
        self.xyz[1] = msg.y
        self.xyz[2] = msg.z

    #test for obs detection
    def goal_callback(self, msg):
        self.goal[0] = msg.x
        self.goal[1] = msg.y
        # print(self.goal)

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
