#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
from path_planning_analysis.interactive import *
import pylab
if __name__=='__main__':
    ax = pylab.axes()
    is_smooth = '-s' in sys.argv

    headers, bags = analysis_argparse(one=False, points=False, arrays=True)
    
    for filename in bags:
        path = PathStats(filename)  
        path.load(True)
        key = path.get_algorithm()
        for header in headers:
            data = getattr(path, header)
            if is_smooth:
                ax.plot(smooth(data, 10))
            else:
                ax.plot(data)
    pylab.show()

