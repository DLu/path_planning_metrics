#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
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
    headers = ['']
    data = []
    for filename in sys.argv[1:]:
        if filename[0]=='-':
            continue
        path = RobotPath(filename)  
        L = len(path.stats())      
        row = [filename]
        for metric in path.stats():
            if len(headers)<=L:
                headers.append(metric.__name__)
            row.append(metric())
        data.append(row)

    if '--summary' in sys.argv:
        m = collections.defaultdict(list)
        
        for row in data:
            key = row[0].split('-')[1]
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

        headers = ['algorithm', 'count'] + headers[1:]

    print_table([headers] + data, False)
