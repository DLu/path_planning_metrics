#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
import os.path
from path_planning_analysis.path_stats import *
from path_planning_analysis.interactive import *
from path_planning_analysis.classes import *
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
    
    paths = [PathStats(filename) for filename in bags]
    constants, parameters = get_parameters([p.features for p in paths])
    
    if len(parameters)==1:
        x_param = parameters[0]
    else:
        x_param = select(parameters, True)[0]
    
    for path in paths:
        row = []

        value = path.features[x_param]        
        if float(value)>limit:
            continue

        stats = path.stats()

        if '--completed' in sys.argv and stats['completed']<1.0:
            continue
        if '--collisions' in sys.argv and stats['collisions']>0.0:
            continue

        for name in headers:
            v = stats[name]
            key = path.features['algorithm']
            xs[key].append( value )
            ys[key].append( v )
    

    if '--box' in sys.argv:
        data = collections.defaultdict(list)
        for key in xs:
            for x,y in zip(xs[key], ys[key]):
                data[x].append(y)

        bxs = sorted(data.keys())
        width = 100
        for a,b in zip(bxs, bxs[1:]):
            width = max(.1, min(b-a, width))

        bys = [data[k] for k in bxs]
        plt.boxplot(bys, positions=bxs, widths=width)
        plt.xlim(bxs[0], bxs[-1])
    else:
        for key in xs:
            ax.plot(xs[key], ys[key], 'o', label=key)
        ax.legend()

    pylab.ylabel(name)
    pylab.xlabel(x_param)
    ax.set_title(map_string(constants))
    
    fig = pylab.gcf()
    fig.canvas.set_window_title(x_param)
    plt.show()

