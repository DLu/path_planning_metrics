#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
import os.path
from path_planning_analysis.path_stats import *
import pylab
import collections
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

def get_fields():
    return ['minimum_distance_to_person']

"""
    if '--basic' in sys.argv:
        headers = ['time', 'collisions', 'completed']
    elif '--update-time' in sys.argv:
        headers = ['min_global_update_time', 'average_global_update_time', 'max_global_update_time', 'min_local_update_time', 'average_local_update_time', 'max_local_update_time']
    elif '--social' in sys.argv:
        headers = ['minimum_distance_to_person', 'average_distance_to_person']
"""

if __name__=='__main__':
    fig, ax = plt.subplots()

    xs = collections.defaultdict(list)
    ys = collections.defaultdict(list)

    headers = get_fields()
    for filename in sys.argv[1:]:
        if filename[0]=='-':
            continue
        path = PathStats(filename)  
        fullpath = os.path.abspath(filename)
        parts = fullpath.split('/')
        algorithm, variable = parts[-2].split('-')
        scenario, value, trial = parts[-1].split('-')

        row = []

        stats = path.stats()
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
        bys = [data[k] for k in bxs]
        plt.boxplot(bys, positions=bxs)
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

