#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
from path_planning_analysis.interactive import *
import pylab

if __name__=='__main__':
    ax = pylab.axes()
    pylab.axis('equal')

    headers, bags = analysis_argparse(one=True, points=False, arrays=True)
    header = headers[0]

    for bagfn in bags:
        path = PathStats(bagfn)
        path.load(True) 
    
        x =[]
        y =[]
        for pose in path.poses:
            x.append(pose.x)
            y.append(pose.y)

        values = getattr(path, header)
        print min(values), max(values)

        ax.scatter(x,y,s=[30*a/max(values) for a in values])

    pylab.show()

