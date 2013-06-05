#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab
if __name__=='__main__':
    filename = sys.argv[1]
    path = RobotPath(filename)
    #path.plot()
    #ts, ds = path.get_displacement()
    v0 = path.get_velocity()
    v1 = smooth(v0, 10)
    pylab.title(filename)
    ax = pylab.axes()
    ax.plot(v0)
    ax.plot(v1)
    pylab.show()
        
