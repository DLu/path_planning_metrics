#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from path_planning_ruler import *
from path_planning_ruler.scenario import Scenario
from path_planning_ruler.move_base import *
import sys
import argparse
import os, errno

def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

def param_keys(s1, s2=None):
    a1 = s1.split('/')
    ns1 = a1[-1]
    if s2 is None:
        return ns1

    a2 = s2.split('/')
    ns2 = a2[-1]

    if ns1 != ns2:
        return ns1, ns2

    i=2
    while i <= len(a1) and i <= len(a2):
        b1 = a1[-i]
        b2 = a2[-i]
        if b1 == b2:
            i+=1
        else:
            return "%s_%s"%(b1, ns1), "%s_%s"%(b2, ns2)

    return '_'.join(a1), '_'.join(a2)

basedir = '/home/dlu/Desktop/path_data'

def multiply(parameterizations, name, val_str):
    vals = int(val_str)
    newp = []
    for p in parameterizations:
        for z in range(vals):
            newm = {name: (vals, z)}
            newm.update(p)
            newp.append(newm)
    return newp
    
if __name__=='__main__':
    rospy.init_node('batch_trials_script')

    parser = argparse.ArgumentParser()
    parser.add_argument('algorithm', metavar='algorithm.cfg')
    parser.add_argument('scenarios', metavar='scenario.yaml', nargs='+')
    parser.add_argument("-n", "--num_trials", dest="n", help='Number of trials per configuration', metavar='N', type=int, default=10)
    parser.add_argument('--clean', action='store_true')
    parser.add_argument('--var1', nargs=2)
    parser.add_argument('--var2', nargs=2)

    args = parser.parse_args()
    scenarios = [Scenario(filename) for filename in args.scenarios]
    parameterizations = [{}]

    if args.var1:
        param1, N_str = args.var1
        parameterizations = multiply(parameterizations, param1, N_str)

        if args.var2:
            param2, N_str = args.var2
            key1, key2 = param_keys(param1, param2)
            parameterizations = multiply(parameterizations, param2, N_str)
            directory = '%(root)s/twoD/%(algorithm)s-%(key1)s-%(key2)s'
            pattern = '%(scenario_key)s-%(value1)s-%(value2)s-%%03d.bag'
        else:
            key1 = param_keys(param1)
            directory = '%(root)s/oneD/%(algorithm)s-%(key1)s'
            pattern = '%(scenario_key)s-%(value1)s-%%03d.bag'
    else:
        directory = '%(root)s/core'
        pattern = '%(scenario_key)s-%(algorithm)s-%%03d.bag'

    m = MoveBaseInstance()

    for parameterization in parameterizations:
        values = m.configure(args.algorithm, parameterization)
        if args.var1:
            value1 = values[param1]
        if args.var2:
            value2 = values[param2]

        m.start()
        algorithm = rospy.get_param('/nav_experiments/algorithm')
        root = basedir

        try:
            for scenario in scenarios:
                scenario_key = scenario.key
                thedir = directory % locals()
                thepattern = pattern % locals()
                
                mkdir_p(thedir)
                full_pattern = '%s/%s'%(thedir, thepattern )
                run_batch_scenario(scenario, args.n, full_pattern, args.clean)
        finally:
            m.shutdown()


