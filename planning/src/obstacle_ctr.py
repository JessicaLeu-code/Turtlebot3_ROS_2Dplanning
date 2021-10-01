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
from geometry_msgs.msg import Twist, PolygonStamped, Point32
from gazebo_msgs.msg import ModelState, ModelStates



def sim():
    rospy.init_node('moving_obstacle_virtual_table')

  #  rospy.init_node('virtual_table')  

    #obstacle_poly_pub = rospy.Publisher('obstacle1_poly', PolygonStamped, queue_size=1)
    #obstacle_v_pub = rospy.Publisher('obstacle1_v', Twist, queue_size=1)
    table_poly_pub = rospy.Publisher('table_poly', PolygonStamped, queue_size=1)
    table_v_pub = rospy.Publisher('table_v', Twist, queue_size=1)
    pub_model =  rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)

    # obstacle_rviz_pub = rospy.Publisher('obstacles', Polygons, queue_size=100)

    vertices = [[1, 1, -1, -1], [5, 2, 2, 5]]
    vel = [[0, 0]]
    obstacle2 = Obstacle(vertices, vel)
    rate = rospy.Rate(10)

   
    t = 0
    
    while not rospy.is_shutdown():
        table_poly_pub.publish(obstacle2.toPolygonStamped())
        table_v_pub.publish(obstacle2.toTwist())
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

                if int(t / 100) % 4 < 2:
                    obstacle.twist.linear.y = -0.6
                else:
                    obstacle.twist.linear.y = 0.6
                pub_model.publish(obstacle)
                t += 1
                time.sleep(0.1)
        rate.sleep()




class Obstacle:
    def __init__(self, vertices=[[0, 0, 1, 1], [0, 1, 1, 0]], vel = [[0,0]]):
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

'''
def main():
    rospy.init_node('moving_obstacle')
    moving = Moving()
'''

if __name__ == '__main__':
   # main()  

    try:
        sim()
     
    except rospy.ROSInterruptException:
        pass
