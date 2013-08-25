#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis.interactive import *
from path_planning_analysis.classes import *
from path_planning_analysis.utils import *
from path_planning_analysis.math_util import *
import pylab
import collections

if __name__=='__main__':
    headers, bags = analysis_argparse()
    precision = 3
    if sys.argv[-1][0:2]=='-p':
        precision = int(sys.argv[-1][2:])
    
    grouping = 'algorithm' if '--summary' in sys.argv else None
    group_data = get_stats(bags, headers, grouping)

    data = rotate_stats(group_data, headers)
    
    pdata = []
    for key in sorted(data):
        row = [key]
        stats = data[key]
        
        if grouping is not None:
            row.append( stats['count'] ) 

        for header in headers:
            arr = stats[header]
            row.append( average(arr) )
        pdata.append(row)

    if grouping is None:
        headers = [''] + headers
    else:
        headers = ['algorithm', 'count'] + headers

    print_table([headers] + pdata, False, precision)
