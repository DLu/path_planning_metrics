import roslib; roslib.load_manifest('path_planning_ruler')
from path_planning_simulation import *
from path_planning_simulation.models import *
import rospy
import yaml
import os.path
import random

class Scenario:
    def __init__(self, filename):
        scenario = yaml.load( open(filename) )
        self.scenario = scenario
        self.start = get_pose2d(scenario, 'start')
        self.goal = get_pose2d(scenario, 'goal')
        self.key = os.path.splitext( os.path.basename(filename) )[0]
        self.objects = scenario.get('objects', {})
        self.spawn_names = []

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    def spawn(self, gazebo ):
        self.spawn_names =[]
        people_names = []
        for name, obj in self.objects.iteritems():
            t = obj.get('type', 'box')
            size = obj.get('size', [1,1,1])
            xyz = obj.get('xyz', [0,0,0])
            rpy = obj.get('rpy', [0,0,0])
            s = '%010x' % random.randrange(16**10)
            name += s

            if t=='box':
                xml = box(name, size, xyz, rpy, is_static=True, plugin='movement' in obj)
            else:
                rospy.logerror("unknown type %s"%t)
                continue
            gazebo.spawn_model(name, xml, get_pose(xyz[0], xyz[1], rpy[2]))
            self.spawn_names.append(name)
            if obj.get('class', '')=='person':
                people_names.append(name)
        rospy.set_param('/nav_experiments/people', people_names)

    def unspawn(self, gazebo):
        for name in self.spawn_names:
            gazebo.delete_model(name)

    def reset(self, gazebo):
        gazebo.set_state('pr2', pose2d_to_pose(self.start))
        rospy.sleep(1.0)

    def get_endpoints(self, t=None):
        if t is None:
            t = rospy.Time.now()

        endpoints = []
        endpoints.append( (t, '/start', self.start) )
        endpoints.append( (t, '/goal' , self.goal ) )

        return endpoints

