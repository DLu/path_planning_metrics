#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis.path_stats import *
from path_planning_analysis.interactive import *
import pylab
import collections

def to_strings(data, precision=3):
    table = []
    for row in data:
        strs = []
        for m in row:
            if type(m)==type(0.0):
                strs.append ( ("%%.%df"%precision) % m )
            else:
                strs.append(str(m))
        table.append(strs)
    return table

def print_table(data, spaces=True, precision=3):
    if spaces:
        for row in to_strings(data, precision):
            print '\t'.join(row)
    else:
        table = to_strings(data, precision)
        lens = [0]*len(table[0])
        for row in table:
            for i, value in enumerate(row):
                v = len(value)
                if v > lens[i]:
                    lens[i] = v

        for row in table:
            s = []
            for i, value in enumerate(row):
                n = lens[i]
                v = str(value)
                s.append( ("%%%ds"%n)%v )
            print '\t'.join(s)

if __name__=='__main__':
    data = []
    
    headers, bags = analysis_argparse()
    precision = 3
    if sys.argv[-1][0:2]=='-p':
        precision = int(sys.argv[-1][2:])
    
    for filename in bags:
        path = PathStats(filename)  
        row = []
        if '--summary' in sys.argv:
            row.append(path.get_algorithm())
        else:
            row.append(filename)

        stats = path.stats()
        if headers is None:
            headers = stats.keys()

        for name in headers:
            row.append(stats[name])

        data.append(row)

    if '--summary' in sys.argv:
        m = collections.defaultdict(list)
        
        for row in data:
            key = row[0]
            m[key].append( row[1:] )
        
        data = []
        for key in sorted(m):
            rows = m[key]
            sums = [0]*len(rows[0])
            for row in rows:
                for i, v in enumerate(row):
                    if v:
                        sums[i] += v
            extra = [key, len(rows)]
            data.append( extra + [a/len(rows) for a in sums])

        headers = ['algorithm', 'count'] + headers
    else:
        headers = [''] + headers

    print_table([headers] + data, False, precision)
