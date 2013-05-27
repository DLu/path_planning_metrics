#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab
if __name__=='__main__':
    filename = sys.argv[1]
    path = RobotPath(filename)
    #path.plot_progress()
    ts, ds = path.get_displacement()
    v0 = path.get_velocity()
    v1 = smooth(v0, 10)

    ax = pylab.axes()
    ax.plot(ts,smooth(derivative(ts, v1),10))
    pylab.show()
        
