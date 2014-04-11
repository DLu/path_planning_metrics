#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_simulation')
import rospy, sys
from path_planning_simulation import GazeboHelper

if __name__=='__main__':
    rospy.init_node('spawn_again')
    g = GazeboHelper()
    print g.get_models()

    if '-m' in sys.argv:
        for arg in sys.argv[1:]:
            m = g.get_models()
            for mod in m:
                if arg in mod:
                    g.delete_model(mod)
    else:
        for arg in sys.argv[1:]:
            g.delete_model(arg)

    if len(sys.argv)>1:
        print g.get_models()
