#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

import numpy as np
import rospy
import time
from geometry_msgs.msg import Twist, PolygonStamped, Point32, PoseStamped
from gazebo_msgs.msg import ModelState, ModelStates
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion

import logging



def sim():
    logger = logging.getLogger("testtest")
    sh = logging.StreamHandler()
    logger.addHandler(sh)

    rospy.init_node('moving_obstacle_virtual_table')

    c = Callback()

  #  rospy.init_node('virtual_table')  

    #obstacle_poly_pub = rospy.Publisher('obstacle1_poly', PolygonStamped, queue_size=1)
    #obstacle_v_pub = rospy.Publisher('obstacle1_v', Twist, queue_size=1)
    table_poly_pub = rospy.Publisher('table_poly', PolygonStamped, queue_size=1)
    table_v_pub = rospy.Publisher('table_v', Twist, queue_size=1)
    pub_model =  rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
    trajectory_pub = rospy.Publisher('trajectory', Path, queue_size=10)

    rospy.Subscriber('/path', Path, c.path_callback)
    rospy.Subscriber('/odom', Odometry, c.odom_callback)


    # obstacle_rviz_pub = rospy.Publisher('obstacles', Polygons, queue_size=100)

    vertices = [[1, 1, -1, -1], [5, 4,  4, 5]]
    vel = [[0, 0]]
    obstacle2 = Obstacle(vertices, vel)
    rate = rospy.Rate(10)

    t = 0

    
    while not rospy.is_shutdown():
        table_poly_pub.publish(obstacle2.toPolygonStamped())
        table_v_pub.publish(obstacle2.toTwist())
        tra = c.toTrajectory()

        if tra is not None:
            trajectory_pub.publish(tra)
        # obstacle_rviz_pub.publish([obstacle.toPolygonStamped()])
        #obstacle.step()
        #rate.sleep()
    
        obstacle = ModelState()
        model = rospy.wait_for_message('gazebo/model_states', ModelStates)



        for i in range(len(model.name)):

            if model.name[i] == 'obstacle':
                obstacle.model_name = 'obstacle'
                obstacle.pose = model.pose[i]
                obstacle.twist = Twist()
                obstacle.twist.angular.z = 0

                #if int(t / 100) % 4 < 2:
                #if int(t / 100) % 4 < 1.5:
                if(t >0):
                    obstacle.twist.linear.x = 0.7

                else:
                    obstacle.twist.linear.y = 0

                #if(t > -100000 and t < -98000):
                #    obstacle.twist.linear.y = 0


                #if(obstacle.pose.position.x <-1):
                #    t = -100000

                #obstacle.twist.linear.x = -0.7

                if(c.start_trg <1):
                    t = 0
                    obstacle.twist.linear.x = 0
                    obstacle.pose.position.x = 0
                    obstacle.pose.position.y = -2
                pub_model.publish(obstacle)
                t += 1

                #logger.info()

                #time.sleep(0.1)
        rate.sleep()




class Obstacle:
    def __init__(self, vertices, vel):
   # def __init__(self, vertices=[[0, 0, 1, 1], [0, 1, 1, 0]], vel=[[0, 0]]):

        self.poly = np.array(vertices) * 1.0
        self.v = np.array(vel).T * 1.0
        self.dt = 0.1
        self.i = 0


    '''
    def step(self):
        if self.i == 150:
            self.v = np.array([[0], [0]])
        if self.i == 190:
            print('MOVE')   
        if self.i == 200:
            self.v = np.array([[0], [0]])
        if self.i == 400:
            self.v = np.array([[0], [0]])
        if self.i == 600:
            self.v = np.array([[0], [0]])

        if self.i > 150 and self.i < 200:
            if np.mod(self.i, 5) == 0:
                self.v = -1*self.v
        self.poly += self.v * self.dt
        self.i += 1 
    '''

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
        print(self.v)
        return t



class Callback:
    def __init__(self):
        self.start_trg = 0
        self.x = [0, 0, 0, 0, 0]
        self.tra_x =[0]
        self.tra_y =[0]
        self.old_x = 0
        self.old_y = 0
        self.tra_num = 0
        self.total_dis = 0

    def odom_callback(self, msg):
        _,_,yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.x = [msg.pose.pose.position.x,
                  msg.pose.pose.position.y,
                  yaw,
                  msg.twist.twist.linear.x,
                  msg.twist.twist.angular.z]

    def path_callback(self, msg):

        if(len(msg.poses) > 0):
            self.start_trg = 1
        return

    def toTrajectory(self):

        dis = np.sqrt(pow(self.old_x-self.x[0], 2) + pow(self.old_y-self.x[1], 2))
        if dis > 0.05 :
            self.tra_x.append(self.x[0])
            self.tra_y.append(self.x[1])
            self.old_x = self.x[0]
            self.old_y = self.x[1]
            self.tra_num = self.tra_num + 1
            self.total_dis = self.total_dis + dis

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




if __name__ == '__main__':
   # main()  

    try:
        sim()
     
    except rospy.ROSInterruptException:
        pass
