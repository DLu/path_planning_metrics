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

import yaml
if __name__=='__main__':
    fig, ax = plt.subplots()

    xs = collections.defaultdict(list)
    ys = collections.defaultdict(list)

    headers, bags = analysis_argparse(one=True, arrays=True)
    header = headers[0]
    
    for filename in bags:
        path = PathStats(filename)  
        path.load(True)
        key = path.get_algorithm()
        if header in path.data_fields:
            ys[key] += getattr(path, header)
        else:
            ys[key].append(path.stats()[header])

    algorithms = sorted(ys.keys())
    data = []
    for key in algorithms:
        data.append(ys[key])

    if '--histo' in sys.argv:
        if len(data)==1:
            tipo = 'stepfilled'
        else:
            tipo = 'step'
        pylab.hist(data, 50, normed=1, histtype=tipo, label=algorithms)
        pylab.legend()
        pylab.xlabel(header)
        pylab.xlabel('Freq')

    else:        
        plt.boxplot(data)
        pylab.ylabel(header)
        pylab.xlabel("Algorithm")
        ax.set_title(header)
        pylab.xticks(range(1, len(algorithms)+1), algorithms)
    
    fig = pylab.gcf()
    fig.canvas.set_window_title(header)
    plt.show()

