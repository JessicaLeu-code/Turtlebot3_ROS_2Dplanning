#!/usr/bin/env python
import rospy
import sys
from find_lines import FindObstacles

if __name__ == '__main__':
    if len(sys.argv) > 3:
        print('Experiment enabled')
        print(str(sys.argv))
        FindObstacles(viz=False, experiment=str(sys.argv[1]))
    else:
        FindObstacles(viz=False)

    rospy.spin()
