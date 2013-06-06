#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from path_planning_ruler import *
import sys

if __name__=='__main__':
    rospy.init_node('test_script')
    start = (0,0,0)
    if len(sys.argv)<=1:
        print "Error no bag specified"
        exit(1)
    filename = sys.argv[1]
    if '.bag' not in filename:
        print "Check args:", filename
        exit(1)
    x = 0
    y = 0
    theta = 0
    if len(sys.argv)>2:
        x = float(sys.argv[2])
        if len(sys.argv)>3:
            y = float(sys.argv[3])
            if len(sys.argv)>4:
                theta = float(sys.argv[4])
    run_empty_room_test(filename, start, (x,y,theta))



