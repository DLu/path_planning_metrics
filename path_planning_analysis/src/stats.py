#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *

if __name__=='__main__':
    filename = sys.argv[1]
    path = RobotPath(filename)
    ts, ds = path.get_displacement()
