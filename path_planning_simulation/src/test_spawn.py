#!/usr/bin/python

from path_planning_simulation import *
from path_planning_simulation.models import box

rospy.init_node('spawner')
g = GazeboHelper()
name = 'david'
xml = box('b')
g.spawn_model(name, xml, get_pose(5,0,0))

