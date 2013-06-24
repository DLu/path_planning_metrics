#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab
if __name__=='__main__':
    ax = pylab.axes()
    is_smooth = '-s' in sys.argv
    
    for filename in sys.argv[1:]:
        if filename[0]=='-':
            continue
        path = RobotPath(filename)
        v0 = path.get_velocity()
        mags = [mag for angle, mag in v0]

        if is_smooth:
            ax.plot(smooth(mags, 10))
        else:
            ax.plot(mags)
    pylab.show()

