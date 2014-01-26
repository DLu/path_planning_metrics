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
    
    if '--summary' in sys.argv:
        merge = True
    else:
        merge = False

    new_headers, data, constants = collect_stats(bags, headers, merge)

    pdata = []
    for key in sorted(data):
        row = []
        stats = data[key]
        
        for header in new_headers:
            v = stats[header]
            if type(v)==list:
                row.append( average(v) )
            else:
                row.append( v )
        pdata.append(row)

    print_table([new_headers] + pdata, False, precision)
    if len(constants)>0:
        print '(%s)'%map_string(constants)
