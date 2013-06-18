#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab

if __name__=='__main__':
    ax = pylab.axes()
    pylab.axis('equal')
    g_path = '-g' in sys.argv
    for filename in sys.argv[1:]:
        if filename[0]=='-':
            continue
        path = RobotPath(filename)  
        path.plot_one(ax)
        if g_path:
            path.plot_global(ax)
    pylab.show()

