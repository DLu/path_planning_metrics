#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
from path_planning_analysis.interactive import *
import pylab
if __name__=='__main__':
    ax = pylab.axes()
    for fn in sys.argv[1:]:
        if '.bag' not in fn:
            continue
        path = PathStats(fn)  
        path.load(True)
        ds = []
        ts = []
        times = [t.to_sec() for t in path.t]
        for i in range(path.get_num_poses()):
            d = path.get_distance_to_goal(i)
            ds.append(d)
            ts.append( path.get_angle_to_goal(i) )
        color = 'red' if 'False' in fn else 'blue'
        ax.plot(times, ds, label=fn, color=color)
        if '-a' in sys.argv:
            ax.plot(times, ts, color=color)
    pylab.xlim(0, 20)
    pylab.axhline(.2, color='r')
    pylab.axhline(.325, color='gray')
    #pylab.axhline(.325*1.5-.05)
    pylab.axhline(.25*1.7, color='orange')
    pylab.axhline(.25*1.7+.325, color='green')
    if '-l' in sys.argv:
        pylab.legend()
    pylab.show()

