import rospy
import yaml

CORE_COSTMAP = '/home/dlu/ros/path_planning_metrics/path_planning_data/core_costmap.yaml'
MOVE_BASE = '/home/dlu/ros/pr2_navigation/pr2_navigation_super_config/params/default_move_base.yaml'
OLD_CONFIGURATION = '/home/dlu/ros/path_planning_metrics/path_planning_data/old_parameters.yaml'
DWA_PARAMS = '/home/dlu/ros/pr2_navigation/pr2_navigation_super_config/params/dwa_planner.yaml'
TPR_PARAMS = '/home/dlu/ros/pr2_navigation/pr2_navigation_config/move_base/base_local_planner_params.yaml'

class Parameter:
    def __init__(self, name):
        self.name = name
        self.value = None
        self.count = None
        self.link = None

    def set_value(self, value):
        self.value = value

    def set_range(self, default, min_v, max_v):
        self.min = min_v
        self.max = max_v
        self.value = default

    def get_value(self, i=None):
        if i is None:
            return self.value
        else:
            spread = self.max - self.min
            return self.min + spread * i / self.count

    def has_constant_value(self):
        return self.count is None

    def has_link(self):
        return self.link is not None

    def set_count(self, count):
        self.count = count

    def set_link(self, link):
        self.link = link        

def parse_args(param_args):
    a = []
    key = None

    if param_args is None:
        return []
    for arg in param_args:
        if key is None:
            key = arg
        else:
            a.append( (key, eval(arg)) )
            key = None
    if key is not None:
        print "Parameter %s has no value"%key

    return a

def is_valid(i, bits):
    for b in bits:
        if i > len(b):
            return False
    return True

def all_unique(s):
    return len(s)==len(set(s))

def param_keys(array):
    bits = []
    for s in array:
        a = s.split('/')[1:]
        bits.append(a)

    last_words = [a[-1] for a in bits]

    if all_unique(last_words):
        return last_words

    print bits
    i=2
    while is_valid(i, bits):
        b = [a[-i] for a in bits]
        if not all_unique(b):
            i+=1
        else:
            return ['%s_%s'%(a[-i], a[-1]) for a in bits]

    return ['_'.join(a) for a in bits]


class Parameterization:
    def __init__(self, algorithm_fn, variables, constants, node_name='move_base_node'):
        config = yaml.load( open(algorithm_fn) )
        self.node_name = node_name
        self.all_params = {}
        self.key_params = []

        self.algorithm = config['algorithm']
        self.all_params['/nav_experiments/algorithm'] = config['algorithm']
        self.all_params['/nav_experiments/topics'] = config['topics']
        self.load_config(CORE_COSTMAP)
        self.load_config(MOVE_BASE)

        self.set_local_planner(config['local_planner'])

        if 'fuerte' in config['algorithm']:
            self.load_config(OLD_CONFIGURATION)
            self.all_params['/move_base_node/local_costmap/footprint_padding'] = 0.015
        else:
            self.load_layers( config['global_layers'], True)
            self.load_layers( config['local_layers'], False)

        self.available_params = {}

        for param in config.get('parameters', []):
            name = param['name']
            p = Parameter(name)
            if 'link' in param:
                p.set_link(param['link'])
            else:
                p.set_range(param['default'], param.get('min', None), param.get('max', None))
            self.available_params[name] = p

        for array, is_constant in [(constants, True), (variables, False)]:
            for key, value in parse_args(array):
                if key in self.available_params:
                    p = self.available_params[key]
                else:
                    p = None
                    for pname in self.available_params:
                        if key in pname:
                            key = pname
                            p = self.available_params[key]
                            break
                    if p is None:
                        self.available_params[key]
                            
                self.key_params.append(key)
                if is_constant:
                    p.set_value(value)
                else:
                    p.set_count(value)

        self.parameterizations = [{}]
        for name, p in self.available_params.iteritems():
            self.multiply(p)

    def load_config(self, filename, ns=None):
        config = yaml.load( open(filename) )
        m = {}
        namespace = '/%s'%self.node_name
        if ns is not None:
            namespace += '/%s'%ns
        for k,v in config.iteritems():
            m["%s/%s"%(namespace,k)] = v
        self.all_params.update(m)

    def multiply(self, param):
        if param.name in self.parameterizations[0]:
            return

        if param.has_link():
            p2 = self.available_params[param.link]
            self.multiply(p2)

        newp = []
        for p in self.parameterizations:
            if param.has_link():
                newm = {param.name: p[p2.name]}
                newm.update(p)
                newp.append(newm)
            elif param.has_constant_value():
                newm = {param.name: param.get_value()}
                newm.update(p)
                newp.append(newm)
            else:
                for z in range(param.count+1):
                    newm = {param.name: param.get_value(z)}
                    newm.update(p)
                    newp.append(newm)
        self.parameterizations = newp

    def set_local_planner(self, name):
        self.all_params['/%s/base_local_planner'%self.node_name] = name

        if 'dwa' in name:
            self.load_config(DWA_PARAMS, ns='DWAPlannerROS')
        elif 'TrajectoryPlanner' in name:
            self.load_config(TPR_PARAMS, ns='TrajectoryPlannerROS')
            rospy.set_param('/%s/move_slow_and_clear/planner_namespace'%self.name, 'TrajectoryPlannerROS')

    def set_params(self, m):
        pname = '/%s'%self.node_name
        if rospy.has_param(pname):
            rospy.delete_param(pname)
        self.set_params_(self.all_params)
        self.set_params_(m)

    def set_params_(self, m):
        for k, v in m.iteritems():
            if k[0]!='/':
                k = '/%s/%s'%(self.node_name, k)
            if rospy.has_param(k):
                v2 = rospy.get_param(k)
                if type(v2)==dict:
                    v3 = {}
                    v3.update(v)
                    v3.update(v2)
                    v = v3

            rospy.set_param(k, v)

    def get_folder(self):
        if len(self.key_params)==0:
            return 'core'
        return '%s-%s'%(self.algorithm, '-'.join( param_keys( sorted( self.key_params) ) ))

    def get_filename(self, scenario, m):
        if len(self.key_params)==0:
            return '%s-%s-%%03d.bag'%(scenario, self.algorithm)

        s = scenario
        for key in sorted( self.key_params ):
            s += '-%s'% str(m[key])
        s += '-%03d.bag'
        return s

    def __repr__(self):
        return '===============\n'.join( [repr(x) for x in self.parameterizations] )
        
    def to_string(self, p):
        s = []
        for k in self.key_params:
            s.append( '%s:%s'%(k, str(p[k])))
        return ' '.join(s)

    def load_layers(self, layers, is_global):
        ns = '/%s/%s_costmap'%(self.node_name, 'global' if is_global else 'local')
        self.all_params['%s/plugins' % ns] = []
        for layer in layers:
            if layer == 'obstacles':
                self.add_standard_obstacle_layer(ns, False)
            elif layer == 'voxels':
                self.add_standard_obstacle_layer(ns, True)
            else:
                self.add_layer(ns, layer)
        
    def add_layer(self, ns, layer_type, layer_name=None, extra=None):
        layers = self.all_params['%s/plugins' % ns]
        if layer_name is None:
            i1 = layer_type.find(':')
            i2 = layer_type.find('Layer', i1)
            layer_name = layer_type[i1+2:i2].lower()
        layers.append( {'name': layer_name, 'type': layer_type} )

        if extra is not None:
            self.all_params['%s/%s'%(ns, layer_name)] = extra

    def add_standard_obstacle_layer(self, ns, voxel_layer=True):
        if voxel_layer:
            name = 'costmap_2d::VoxelLayer'
        else:
            name = 'costmap_2d::ObstacleLayer'

        self.add_layer(ns, name, 'obstacles', yaml.load(open('/home/dlu/ros/path_planning_metrics/path_planning_data/obstacles.yaml')))

