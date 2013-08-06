#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis.path_stats import *
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

def print_table(data, spaces=True):
    if spaces:
        for row in to_strings(data):
            print '\t'.join(row)
    else:
        table = to_strings(data)
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
    
    headers = None
    if '--basic' in sys.argv:
        headers = ['time', 'collisions', 'completed']
    elif '--update-time' in sys.argv:
        headers = ['min_global_update_time', 'average_global_update_time', 'max_global_update_time', 'min_local_update_time', 'average_local_update_time', 'max_local_update_time']
    elif '--social' in sys.argv:
        headers = ['minimum_distance_to_person', 'average_distance_to_person']
    
    for filename in sys.argv[1:]:
        if filename[0]=='-':
            continue
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
                    sums[i] += v
            extra = [key, len(rows)]
            data.append( extra + [a/len(rows) for a in sums])

        headers = ['algorithm', 'count'] + headers
    else:
        headers = [''] + headers

    print_table([headers] + data, False)
