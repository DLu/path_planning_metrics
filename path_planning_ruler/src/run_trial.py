#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from path_planning_ruler import *
from path_planning_ruler.scenario import Scenario
import sys

if __name__=='__main__':
    rospy.init_node('test_script')
    scenario = Scenario(sys.argv[1])
    filename = sys.argv[2]

    run_scenario(scenario, filename)

