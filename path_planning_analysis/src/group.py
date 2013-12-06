#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis.interactive import *
from path_planning_analysis.classes import *
from path_planning_analysis.utils import *
from path_planning_analysis.math_util import *
import pylab
import collections

if __name__=='__main__':
    bags = sys.argv[1:]

    groups = collections.defaultdict(list)
    i = 0
    s0 = ''
    for filename in bags:
        s = '%.2f'%(float(i)/len(bags))
        if s!=s0:
            print s
            s0 = s
        i+=1
        path = PathStats(filename)  
        stats = path.stats()
        params = path.get_params()

        if stats['collisions'] > 0:
            if params['occdist_scale']==0.0:
                groups['collisions'].append(path)
            else:
                groups['weird_collisions'].append(path)
        elif stats['completed']==1.0:
            if stats['time']<33:
                groups['quick'].append(path)
            else:
                groups['slow'].append(path)
        elif params['goal_distance_bias']==0 and params['path_distance_bias']==0:
            groups['no_incentive'].append(path)
        elif stats['distance_to_goal'] < .5:
            if params['path_orientation_bias']>0.0:
                groups['endgame_prob'].append(path)
            else:
                groups['other_endgame'].append(path)
        elif stats['distance_to_goal'] > 1.5 and stats['distance_to_goal'] < 4.0:
            if params['goal_distance_bias']==0.0:
                groups['halfway0'].append(path)
            else:
                groups['halfway?'].append(path)
        else:
            groups['other'].append(path)

    if True:
        for group, paths in groups.iteritems():
            print "=========%s================="%group
            for path in sorted(paths):
                print path.filename
            print
        print

    for group, paths in groups.iteritems():
        count = len(paths)
        pct = float(count)*100/len(bags)
        pcts = "%d%%"%pct
        
        print '%-15s %04d %3s'%(group, count, pcts)
