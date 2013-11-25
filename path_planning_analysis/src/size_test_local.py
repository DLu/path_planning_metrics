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
import rosbag
 
import yaml
if __name__=='__main__':
    fig, ax = plt.subplots()

    xs = collections.defaultdict(list)

    algorithms = set()
    keys = set()

    for filename in sys.argv[1:]:
        if '.bag' not in filename:
            continue

        a = []
        bag = rosbag.Bag(filename)
        alg, val, trial = filename.split('_')

        for topic, msg, t in bag.read_messages():
            a.append(msg.data)


        algorithms.add(alg)
        keys.add(val)
        print filename, alg
        xs[ (alg, val) ].append( sum(a)/len(a) )

    for alg in algorithms:
        x = []
        y = []
        for key in keys:
            if (alg, key) in xs:
                x.append( float(key) )

        x.sort()
        x2 = []
        for xi in x:
            key = "%.1f"%xi
            vs = xs[ (alg, key) ]
            v = sum(vs)/len(vs)
            y.append(v)
            #x2.append(pow(xi,.5))
        plt.plot(x, y, marker='o', label=alg)
            

#    pylab.ylabel(header)
    pylab.xlabel("Algorithm")
#    ax.set_title(header)
    #pylab.xticks([2*x-0.5 for x in range(1, len(keys)+1)], keys)
    pylab.legend()
    fig = pylab.gcf()
#    fig.canvas.set_window_title(header)
    plt.show()

