import roslib; roslib.load_manifest('path_planning_ruler')
from path_planning_simulation import *
from path_planning_simulation.models import *
import rospy
import yaml
import os.path
from geometry_msgs.msg import Pose2D
import random

def eval_s(val, params):
    if type(val)==float or type(val)==int:
        return val
    for param in params:
        if param in val:
            val = val.replace(param, str(params[param]))
    try:                
        return eval(val)
    except NameError, e:
        raise NameError('%s (%s)'%(str(e), val))

def get_pose2d(scenario, key, params):
    pose = Pose2D()
    if key in scenario:
        value = scenario[key]
        if type(value)==type([]):
            pose.x =     eval_s(value[0], params)
            pose.y =     eval_s(value[1], params)
            pose.theta = eval_s(value[2], params)
        else:
            print "Unknown type"
    return pose

def pose2d_to_pose(pose):
    return get_pose(pose.x, pose.y, pose.theta)
    
def get_triple(obj, name, default, params):
    if name not in obj:
        return default
    val = []
    for a in obj[name]:
        val.append( eval_s(a, params) )
    return val
    
def to_triple(obj):
    return [obj.x, obj.y, obj.theta]
    
class GazeboObject:
    def __init__(self, m, params={}):
        self.type = m.get('type', 'box')
        self.size = get_triple(m, 'size', [1,1,1], params)
        self.xyz = get_triple(m, 'xyz', [0,0,0], params)
        self.rpy = get_triple(m, 'rpy', [0,0,0], params)
        self.movement = m.get('movement', None)
        self.is_person = m.get('class', '')=='person'
        self.spawn_name = None
        
    def get_spawn_name(self):
        if self.spawn_name is None:
            s = '%010x' % random.randrange(16**10)
            self.spawn_name = self.name + s
            
        return self.spawn_name
        
    def get_map(self):
        m = {'type': self.type, 'size': self.size, 'xyz': self.xyz, 'rpy': self.rpy}
        if self.movement:
            m['movement'] = self.movement
        if self.is_person:
            m['class'] = 'person'
        return m

class Scenario:
    def __init__(self, filename, params={}):
        scenario = yaml.load( open(filename) )
        self.start = get_pose2d(scenario, 'start', params)
        self.goal = get_pose2d(scenario, 'goal', params)
        self.key = os.path.splitext( os.path.basename(filename) )[0]
        self.objects = {}
        for name, obj in scenario.get('objects', {}).iteritems():
            self.objects[name] = GazeboObject(obj, params)
        
        self.spawn_names = []

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    def spawn(self, gazebo ):
        self.spawn_names =[]
        people_names = []
        for name, obj in self.objects.iteritems():
            if t=='box':
                xml = box(obj.get_spawn_name(), obj.size, obj.xyz, obj.rpy, is_static=True, plugin=(obj.movement is not None))
            else:
                rospy.logerror("unknown type %s"%t)
                continue
            gazebo.spawn_model(name, xml, get_pose(obj.xyz[0], obj.xyz[1], obj.rpy[2]))
            
            if obj.is_person:
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
        
    def get_scenario(self):
        m = {'start': to_triple(self.start), 'goal': to_triple(self.goal)}
        o = {}
        for name, obj in self.objects.iteritems():
            o[name] = obj.get_map()
        if len(o)>0:
            m['objects'] = o
        return m
        
    def __repr__(self):
        return yaml.dump(self.get_scenario(), default_flow_style=False)

