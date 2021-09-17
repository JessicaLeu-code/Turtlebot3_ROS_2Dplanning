#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PolygonStamped, Point, Point32
from nav_msgs.msg import Path, Odometry
from pathmodule import Follower
from std_msgs.msg import Float32

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

def follow_waypoints():
    r = Follower()

    rospy.init_node('tracker')#rospy.Subscriber('/filteredGPS', Point, r.filtercall_back)
    rospy.Subscriber('filteredGPS', Point, r.filter_callback)

    rospy.Subscriber('/path', Path, r.path_callback)
    rospy.Subscriber('/odom', Odometry, r.odom_callback)
    rospy.Subscriber('/obstacle1_poly', PolygonStamped, r.obstacle_poly_callback)
    rospy.Subscriber('/obstacle1_v', Twist, r.obstacle_v_callback)
    rospy.Subscriber('/dt', Float32, r.dt_callback)

    cmv_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    waypoint_pub = rospy.Publisher('waypoints', Path, queue_size=10)
    waypoint1_pub = rospy.Publisher('w2', Path, queue_size=10) 
    r.mode = rospy.get_param('~tracking_mode')
    r.avg = rospy.get_param('~look_ahead_mode')
    r.lookahead = rospy.get_param('~look_ahead_dist')

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        '''
                if os.name != 'nt':
                    settings = termios.tcgetattr(sys.stdin)
                key = getKey()

                if key == 's' :
            
                    cmv_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
                twist = Twist()
                twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
                cmv_pub.publish(twist)
                    print "STOP!! "
            while(1):
            print "STOP!! "
        '''

        cmv = r.move_collision_detection()      # which waypoint follower to use
        pts = r.toPath(r.waypoints)
        pts2 = r.toPath(r.waypoints2)
        if cmv is not None:
            cmv_pub.publish(cmv)
        if pts is not None:
            waypoint_pub.publish(pts)
        if pts2 is not None:
            waypoint1_pub.publish(pts2)
        rate.sleep()

# to stop running
def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


    

if __name__ == '__main__':
    
    try:
        follow_waypoints()
    except rospy.ROSInterruptException:
        pass
