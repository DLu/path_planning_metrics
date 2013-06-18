#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab

if __name__=='__main__':
    table = []
    cols = ['algorithm', 'tests', 'completion', 'collisions', 'time']
    table.append(cols)

    m = collections.defaultdict(list)

    for filename in sys.argv[1:]:
        path = RobotPath(filename)  
        key = path.get_algorithm()

        if not path.valid:
            print "INVALID %s"%filename
            continue
            
        data = []
        data.append( path.completed() )
        data.append( path.collisions() )
        data.append( path.time() )
        m[key].append(data)

    for key in sorted(m):
        rows = m[key]
        sums = [0]*len(rows[0])
        for row in rows:
            for i, v in enumerate(row):
                sums[i] += v
        extra = [key, len(rows)]
        table.append( extra + [a/len(rows) for a in sums])


    lens = [0]*len(cols)
    for row in table:
        for i, value in enumerate(row):
            v = len(str(value))
            if v > lens[i]:
                lens[i] = v

    for row in table:
        s = []
        for i, value in enumerate(row):
            n = lens[i]
            v = str(value)
            s.append( ("%%%ds"%n)%v )
        print '\t'.join(s)
