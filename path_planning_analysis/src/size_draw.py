#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab

if __name__=='__main__':
    ax = pylab.axes()
    pylab.axis('equal')
    path = RobotPath(sys.argv[1])  
    
    x =[]
    y =[]
    for t, pose in path.poses:
        x.append(pose.x)
        y.append(pose.y)
    curvatures = path.get_curvatures(5,2)

    ax.scatter(x,y,s=[30*a for a in curvatures])

    pylab.show()

