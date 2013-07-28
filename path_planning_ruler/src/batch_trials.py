#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from path_planning_ruler import *
from path_planning_ruler.scenario import Scenario
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
            parameterizations = multiply(parameterizations, param2, N_str)
            directory = '%(basedir)s/twoD/%(algorithm)s-%(param1)s-%(param2)s'
            pattern = '%(scenario_key)s-%(value1)s-%(value2)s-%%03d.bag'
        else:
            directory = '%(basedir)s/oneD/%(algorithm)s-%(param1)s'
            pattern = '%(scenario_key)s-%(value1)s-%%03d.bag'
    else:
        directory = '%(basedir)s/core'
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

        for scenario in scenarios:
            mkdir_p(directory)
            full_pattern = '%s/%s'%(directory, pattern % locals() )
            run_batch_scenario(scenario, full_pattern, args.n, args.clean)

        m.shutdown()


