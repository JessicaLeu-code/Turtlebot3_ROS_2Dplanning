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
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState, ModelStates

class Moving():
    def __init__(self):
        self.pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.moving()

    def moving(self):
        t = 0
        while not rospy.is_shutdown():
            obstacle = ModelState()
            model = rospy.wait_for_message('gazebo/model_states', ModelStates)
            for i in range(len(model.name)):
                if model.name[i] == 'obstacle':
                    obstacle.model_name = 'obstacle'
                    obstacle.pose = model.pose[i]
                    obstacle.twist = Twist()
                    obstacle.twist.angular.z = 0.2

                    # # move back and forth
                    if int(t / 50) % 2 == 0:
                        # omega = -np.pi / 120
                        # r = 30
                        # theta = np.pi + omega * t
                        # r_vec = np.array([r*np.cos(theta), r*np.sin(theta), 0])
                        # w_vec = np.array([0, 0, omega])
                        # v = np.cross(w_vec, r_vec)
                        # print("r", r_vec)
                        # print("w", w_vec)
                        # print("v", v)
                        # obstacle.twist.linear.x = v[0]
                        # obstacle.twist.linear.y = v[1]
                        obstacle.twist.linear.y = -0.5
                    elif int(t / 50) % 2 == 1:
                        obstacle.twist.linear.y = 0.5

                    if t < 30:
                        obstacle.twist.linear.y = 0
                        obstacle.twist.linear.x = 0
                    self.pub_model.publish(obstacle)
                    t += 1
                    time.sleep(0.1)

def main():
    rospy.init_node('moving_obstacle')
    moving = Moving()

if __name__ == '__main__':
    main()
