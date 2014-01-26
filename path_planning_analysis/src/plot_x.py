#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
from path_planning_analysis.interactive import *
import pylab

#TODO: Add functionality to draw these lines again
#LINES = [(.2, 'r'), (.324, 'gray'), (.25*1.7, 'orange'), (.25*1.7+.325, 'green')]
#for value, color in LINES:
#        pylab.axhline(value, color=color)
#TODO add limit pylab.xlim(0, 20)

if __name__=='__main__':
    ax = pylab.axes()
    is_smooth = '-s' in sys.argv

    headers, bags = analysis_argparse(one=False, points=False, arrays=True)
    
    for filename in bags:
        path = PathStats(filename)  
        path.load(True)
        key = path.get_algorithm()
        times = [t.to_sec() for t in path.t]
        for header in headers:
            data = getattr(path, header)
            if is_smooth:
                ax.plot(times, smooth(data, 10), label=header)
            else:
                ax.plot(times, data, label=header)
                
    if '-l' in sys.argv:
        pylab.legend()
    pylab.show()

