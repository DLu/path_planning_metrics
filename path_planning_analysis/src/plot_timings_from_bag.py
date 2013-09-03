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

remaps = {
    'Voxels/UpdateOrigin': 'UpdateOrigin',
    'Obstacles/FootprintCosts': 'Clear Footprint',
    'Voxels/FootprintBounds': 'Clear Footprint',
    'Voxels/Raytrace': 'Raytrace',
    'Voxels/Mark': 'Mark',
    'Update Obstacles': 'Mark',
    'Clear Markers': 'Reset',
    'Map Reset': 'Reset',
    'Inflation/CostScan': 'Reset',
    'Obstacles/UpdateCosts': 'Mark',
    'Voxels/Setup': 'Initialization',
    'Obstacles/FootprintBounds': 'Clear Footprint',
    'Obstacles/Raytrace': 'Raytrace',
    'Obstacles/Mark': 'Mark',
    'Obstacles/UpdateOrigin': 'UpdateOrigin',
    'Obstacles/Setup': 'Initialization',
    'Static/UpdateBounds': 'Static',
    'Static/UpdateCosts': 'Static'
}
 
import yaml
if __name__=='__main__':
    fig, ax = plt.subplots()

    xs = collections.defaultdict(list)
    scope = 'local'

    algorithms = set()
    keys = set()

    for filename in sys.argv[1:]:
        if '.bag' not in filename:
            continue

        bag = rosbag.Bag(filename)
        alg = filename
        algorithms.add(alg)

        for topic, msg, t in bag.read_messages():
            if 'cycle_times' in topic:
                cdata = collections.defaultdict(float)
                for event in msg.events:
                    name = event.name
                    time = event.time
                    key = name
                    if key in remaps:
                        key = remaps[key]

                    keys.add(key)
                    cdata[key] += time
                for key, value in cdata.iteritems():
                    xs[ (alg, key) ].append(value)
    algorithms = sorted(algorithms)
    keys = sorted(keys)

    data = []
    for key in keys:
        for alg in algorithms:
            data.append(xs[(alg,key)])

    b = plt.boxplot(data)
    print b.keys()

    for field in ['boxes', 'medians']:
        flag = True
        for line in b[field]:
            if flag:
                line.set_color('r')
            else:
                line.set_color('b')
            flag = not flag


#    pylab.ylabel(header)
    pylab.xlabel("Algorithm")
#    ax.set_title(header)
    pylab.xticks([2*x-0.5 for x in range(1, len(keys)+1)], keys)
    
    fig = pylab.gcf()
#    fig.canvas.set_window_title(header)
    plt.show()

