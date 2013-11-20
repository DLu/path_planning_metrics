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

if __name__=='__main__':
    fig, ax = plt.subplots()

    xs = collections.defaultdict(list)
    ys = collections.defaultdict(list)

    headers, bags = analysis_argparse(one=True)
    limit = 1e80
    
    for filename in bags:

        fullpath = os.path.abspath(filename)
        parts = fullpath.split('/')
        algorithm, variable = parts[-2].split('-')
        scenario, value, trial = parts[-1].split('-')

        if float(value)>limit:
            continue
        
        path = PathStats(filename)  
        row = []

        stats = path.stats()

        if '--completed' in sys.argv and stats['completed']<1.0:
            continue
        if '--collisions' in sys.argv and stats['collisions']>0.0:
            continue

        for name in headers:
            v = stats[name]
            key = algorithm
            xs[key].append( float(value) )
            ys[key].append( v )
    

    if '--box' in sys.argv:
        data = collections.defaultdict(list)
        for key in xs:
            for x,y in zip(xs[key], ys[key]):
                data[x].append(y)

        bxs = sorted(data.keys())
        width = 100
        for a,b in zip(bxs, bxs[1:]):
            width = min(b-a, width)

        bys = [data[k] for k in bxs]
        plt.boxplot(bys, positions=bxs, widths=width)
    else:
        for key in xs:
            ax.plot(xs[key], ys[key], 'o', label=key)
        ax.legend()

    pylab.ylabel(name)
    pylab.xlabel(variable)
    ax.set_title(scenario)
    
    fig = pylab.gcf()
    fig.canvas.set_window_title(variable)
    plt.show()

