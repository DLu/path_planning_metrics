#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from path_planning_ruler import *
from path_planning_ruler.scenario import Scenario
from path_planning_ruler.move_base import *
from path_planning_ruler.parameterization import *
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

def run_one_set(parameterization, scenarios, n, clean, quiet):
    m = MoveBaseInstance(quiet=quiet)
    scenarios = [Scenario(filename) for filename in scenarios]

    for p in parameterization.parameterizations:
        parameterization.set_params(p)
        
        if len(parameterization.parameterizations)>1:
            rospy.loginfo( parameterization.to_string(p) )

        for scenario in scenarios:
            thedir = '%s/%s'%(basedir, parameterization.get_folder())            
            mkdir_p(thedir)
            
            full_pattern = '%s/%s'%(thedir, parametyerization.get_filename(p) )
            run_batch_scenario(m, scenario, n, full_pattern, clean, quiet)
    
if __name__=='__main__':
    rospy.init_node('batch_trials_script')

    parser = argparse.ArgumentParser()
    parser.add_argument('algorithm', metavar='algorithm.cfg')
    parser.add_argument('scenario', metavar='scenario.yaml', nargs='+')
    parser.add_argument('-v', dest='variables', nargs="+", type=str)
    parser.add_argument('-c', dest='constants', nargs="+", type=str)
    parser.add_argument("-n", "--num_trials", dest="n", help='Number of trials per configuration', metavar='N', type=int, default=10)
    parser.add_argument('--clean', action='store_true')
    parser.add_argument('-q', '--quiet', action='store_true')

    list_of_args = [] 
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
            list_of_args.append( parser.parse_args(line.split()) )
        f.close()        
    else:
        list_of_args.append( parser.parse_args() )
        
    for args in list_of_args:
        parameterization = Parameterization(args.algorithm, args.variables, args.constants)
        run_one_set(parameterization, args.scenarios, args.n, args.clean, args.quiet)

