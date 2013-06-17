#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from path_planning_ruler import *
from path_planning_ruler.scenario import Scenario
import sys

if __name__=='__main__':
    rospy.init_node('test_script')

    scenarios = []
    n = 10
    clean = False

    for arg in sys.argv[1:]:
        if '.yaml' in arg:
            scenarios.append(arg)
        elif '--clean'==arg:
            clean = True
        else:
            n = int(arg)

    for filename in scenarios:
        scenario = Scenario(filename)
        run_batch_scenario(scenario, n, '/home/dlu/Desktop/path_data', clean)


