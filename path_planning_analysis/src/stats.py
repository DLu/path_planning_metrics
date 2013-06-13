#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis import *
import pylab

if __name__=='__main__':
    headers = []
    data = []
    for filename in sys.argv[1:]:
        path = RobotPath(filename)  
        L = len(path.stats())      
        row = [filename]
        for metric in path.stats():
            if len(headers)<L:
                headers.append(metric.__name__)
            row.append(metric())
        data.append(row)

    s = '\t'
    for h in headers:
        s+= '%s\t'%h
    print s
    for row in data:
        s = ''
        for m in row:
            if type(m)==type(0.0):
                s+= '%.3f\t'%m
            else:
                s += '%s\t'%m
        print s
