#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
import os.path
from path_planning_analysis.path_stats import *
from path_planning_analysis.interactive import *
import pylab
import collections
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
    
def clean_text(s):
    return ' '.join(s.split('_')).title()

def get_labels(array):
    N = len(array)
    if N<=5:
        return ([a+.5 for a in range(N)], ["%.2f"%a for a in array])
    else:
    
        return ( [0, N], [array[0], array[N-1]])

if __name__=='__main__':
    fig, ax = plt.subplots()

    data = collections.defaultdict(list)
    xs = set()
    ys = set()

    headers, bags = analysis_argparse(one=True)
    header = headers[0]
    vs = []
    
    for filename in bags:
        path = PathStats(filename)  
        fullpath = os.path.abspath(filename)
        parts = fullpath.split('/')
        algorithm, variable1, variable2 = parts[-2].split('-')
        scenario, value1, value2, trial = parts[-1].split('-')
        if '--reverse' in sys.argv:
            t = variable1, variable2, value1, value2
            variable2, variable1, value2, value1 = t

        row = []

        stats = path.stats()

        if '--completed' in sys.argv and stats['completed']<1.0:
            continue

        v = stats[header]
        key = float(value1), float(value2)
        xs.add(key[0])
        ys.add(key[1])
        vs.append( float(v) )
        data[key].append( float(v) )

    colors = []
    if '-b' in sys.argv:
        colors += [('black')]
    if '-c' in sys.argv:
        colors += [(pylab.cm.jet(i)) for i in xrange(1,255)]
    if '-w' in sys.argv:
        colors += [('white')]

    if len(colors)==0:
        colors += [(pylab.cm.jet(i)) for i in xrange(1,255)]
    

    new_map = matplotlib.colors.LinearSegmentedColormap.from_list('new_map', colors, N=256)

    xs = sorted(list(xs))
    ys = sorted(list(ys))
    array = []
    for y in sorted(ys):
        row = []
        for x in sorted(xs):
            m = data[(x,y)]
            row.append( average(m) )
        array.append(row)    

    variable1 = clean_text(variable1)
    variable2 = clean_text(variable2)
    header = clean_text(header)

    mdata = pylab.array(array) 
    pylab.pcolor(mdata, cmap=new_map)

    cbar = pylab.colorbar()
    cbar.set_label(header)

    pylab.xlabel(variable1)
    labels = get_labels(xs)
    pylab.xticks(labels[0], labels[1]) 
    
    pylab.ylabel(variable2)
    labels = get_labels(ys)
    pylab.yticks(labels[0], labels[1])
    
    s = '%s: %s\n%s'%(variable1, variable2, header)
    pylab.title(s)

    fig = pylab.gcf()
    fig.canvas.set_window_title(s.replace('\n', '--'))
    plt.show()

