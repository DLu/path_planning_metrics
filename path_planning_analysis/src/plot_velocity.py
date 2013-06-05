#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab
if __name__=='__main__':
    ax = pylab.axes()
    for filename in sys.argv[1:]:
        path = RobotPath(filename)
        v0 = path.get_velocity()
        v1 = smooth(v0, 10)
        ax.plot(v1)
    pylab.show()
        
