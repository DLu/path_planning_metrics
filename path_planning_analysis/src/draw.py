#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab

if __name__=='__main__':
    ax = pylab.axes()
    pylab.axis('equal')
    for filename in sys.argv[1:]:
        path = RobotPath(filename)  
        path.plot_one(ax)
    pylab.show()

