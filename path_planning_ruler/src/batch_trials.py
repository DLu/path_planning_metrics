#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from path_planning_ruler import *
from path_planning_ruler.scenario import Scenario
from path_planning_ruler.move_base import *
from path_planning_ruler.parameterization import *
from twilio_ros import send_text
import sys
import argparse
import os, errno
import collections

def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

basedir = '/home/dlu/Desktop/path_data'

def run_one_set(parameterization, scenarios, n, clean, quiet):
    m = MoveBaseInstance(name=parameterization.node_name, quiet=quiet)
    all_stats = []

    for p in parameterization.parameterizations:
        parameterization.set_params(p)
        
        if len(parameterization.parameterizations)>1:
            rospy.loginfo( parameterization.to_string(p) )

        for scenario_fn in scenarios:
            scenario = Scenario(scenario_fn, p)
            thedir = '%s/%s'%(basedir, parameterization.get_folder())            
            mkdir_p(thedir)
            
            full_pattern = '%s/%s'%(thedir, parameterization.get_filename(scenario.key, p) )
            stats = run_batch_scenario(m, scenario, n, full_pattern, clean, quiet)
            all_stats.append(stats)

    return all_stats
    
if __name__=='__main__':
    rospy.init_node('batch_trials_script')

    text = False
    parser = argparse.ArgumentParser()
    parser.add_argument('algorithm', metavar='algorithm.cfg')
    parser.add_argument('scenarios', metavar='scenario.yaml', nargs='+')
    parser.add_argument('-v', dest='variables', nargs="+", type=str)
    parser.add_argument('-c', dest='constants', nargs="+", type=str)
    parser.add_argument("-n", "--num_trials", dest="n", help='Number of trials per configuration', metavar='N', type=int, default=10)
    parser.add_argument('--clean', action='store_true')
    parser.add_argument('-q', '--quiet', action='store_true')
    parser.add_argument('-t', '--text', action='store_true')

    list_of_args = [] 
    if '-b' in sys.argv:
        p2 = argparse.ArgumentParser()
        p2.add_argument('-b', '--batch', dest="batchfile")
        p2.add_argument('-q', '--quiet', action='store_true')
        p2.add_argument('-c', '--clean', action='store_true')
        p2.add_argument('-t', '--text', action='store_true')

        a2 = p2.parse_args()
        text = args.text
        f = open(a2.batchfile, 'r')
        for line in f.readlines():
            if len(line.strip())==0:
                continue
            list_of_args.append( parser.parse_args(line.split()) )
        f.close()        
    else:
        args = parser.parse_args() 
        text = args.text
        list_of_args.append( args )
        
    stats = []
    for args in list_of_args:
        parameterization = Parameterization(args.algorithm, args.variables, args.constants)
        stats += run_one_set(parameterization, args.scenarios, args.n, args.clean, args.quiet) 

    collected = collections.defaultdict(int)
    for stat in stats:
        for k, v in stat.iteritems():
            collected[k] += v


    import socket
    host = socket.gethostname()
    ready = collected['total']-collected['to_run']+collected['run']
    s = "Ran %d/%d tests (%d/%d) on %s"%(collected['run'], collected['to_run'], ready, collected['total'], host)

    if text:
        send_text('+18455271217', s)
    else:
        print s

