#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_ruler')
import rospy
from path_planning_ruler import *
from path_planning_ruler.scenario import Scenario
from path_planning_ruler.move_base import *
from path_planning_ruler.parameterization import *
from path_planning_ruler.batch_arg_parser import parse_args
from path_planning_ruler.utils import *
from std_srvs.srv import *
import collections
import sys

basedir = '/home/dlu/Desktop/path_data'
if __name__=='__main__':
    rospy.init_node('test_scenario')

    list_of_args, should_text = parse_args()
    print "Waiting for animator"
    rospy.wait_for_service('/animator/reset')
    resetter = rospy.ServiceProxy('/animator/reset', Empty)

    # Run the tests
    for args in list_of_args:
        parameterization = Parameterization(args.algorithm, args.scenario, args.variables, args.constants, basedir)

        for p in parameterization.parameterizations:
            parameterization.set_params(p)
            scenario = parameterization.scenario
            g = GazeboHelper(args.quiet)
            
            try:
                g.spawn_robot_maybe()
                scenario.spawn(g)
                scenario.reset(g)
                resetter()

                raw_input("Press key to continue")

            except rospy.service.ServiceException, e:
                rospy.logerr("SERVICE EXCEPTION: %s" % str(e) )
            finally:
                scenario.unspawn(g)
                print "Unspawned"


