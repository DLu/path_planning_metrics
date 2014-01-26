#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab

if __name__=='__main__':
    ax = pylab.axes()
    pylab.axis('equal')
    path = RobotPath(sys.argv[1])  
    vels = path.get_velocity()

    x =[]
    y =[]
    for t, pose in path.poses:
        x.append(pose.x)
        y.append(pose.y)
    ax.plot(path.poses[0][1].x,path.poses[0][1].y,color="white")
    ax.plot(path.poses[-1][1].x*1.1,path.poses[-1][1].y*1.1,color="white")
    S = 2


    for i, (t, pose) in enumerate(path.poses):
        if i % 4 != 1:
            continue
        theta = pose.theta #+ pi
        dx = cos(theta) / 500
        dy = sin(theta) / 500

        ax.arrow(pose.x, pose.y, dx, dy, head_width=S*.025, head_length=S*.05, fc='red')  

        theta, amp = vels[i]
        dx = cos(theta) / 500
        dy = sin(theta) / 500
        ax.arrow(pose.x, pose.y, dx, dy, head_width=S*.005, head_length=S*.25*amp, fc='blue')  
    pylab.show()

