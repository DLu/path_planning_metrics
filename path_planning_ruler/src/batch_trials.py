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

def parameterize(var1, var2):
    parameterizations = [{}]
    param1 = param2 = key1 = key2 = None

    if var1:
        param1, N_str = var1
        parameterizations = multiply(parameterizations, param1, N_str)

        if var2:
            param2, N_str = var2
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

    return parameterizations, directory, pattern, param1, key1, param2, key2


def run_one_set(algorithm_fn, scenarios, n, parameterizations, directory, pattern, param1, key1, param2, key2, clean, quiet):
    m = MoveBaseInstance(quiet=quiet)
    scenarios = [Scenario(filename) for filename in scenarios]

    for parameterization in parameterizations:
        values = m.configure(algorithm_fn, parameterization)
        if param1:
            value1 = values[param1]
        if param2:
            value2 = values[param2]
        if len(parameterizations)>1:
            s = ', '.join(['%s: %s'%(str(k),str(v)) for k,v in values.iteritems()])
            rospy.loginfo(s)

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
                run_batch_scenario(scenario, n, full_pattern, clean, quiet)
        finally:
            m.shutdown()


    
if __name__=='__main__':
    rospy.init_node('batch_trials_script')

    parser = argparse.ArgumentParser()
    parser.add_argument('algorithm', metavar='algorithm.cfg')
    parser.add_argument('scenarios', metavar='scenario.yaml', nargs='+')
    parser.add_argument("-n", "--num_trials", dest="n", help='Number of trials per configuration', metavar='N', type=int, default=10)
    parser.add_argument('--clean', action='store_true')
    parser.add_argument('--var1', nargs=2)
    parser.add_argument('--var2', nargs=2)
    parser.add_argument('-q', '--quiet', action='store_true')

    if '-b' in sys.argv:
        p2 = argparse.ArgumentParser()
        p2.add_argument('-b', '--batch', dest="batchfile")
        p2.add_argument('-q', '--quiet', action='store_true')
        p2.add_argument('-c', '--clean', action='store_true')
        a2 = p2.parse_args()
        f = open(a2.batchfile, 'r')
        for line in f.readlines():
            if len(line.strip())==0:
                continue
            args = parser.parse_args(line.split())
            parameterizations, directory, pattern, param1, key1, param2, key2 = parameterize(args.var1, args.var2)
            run_one_set(args.algorithm, args.scenarios, args.n, parameterizations, directory, pattern, param1, key1, param2, key2, args.clean or a2.clean, a2.quiet or args.clean)
        f.close()        
    else:

        args = parser.parse_args()
        parameterizations, directory, pattern, param1, key1, param2, key2 = parameterize(args.var1, args.var2)
        run_one_set(args.algorithm, args.scenarios, args.n, parameterizations, directory, pattern, param1, key1, param2, key2, args.clean, args.quiet)



