from path_planning_ruler.scenario import Scenario
import rospy
import yaml

CORE_COSTMAP = '/home/dlu/ros/path_planning_metrics/path_planning_data/core_costmap.yaml'
MOVE_BASE = '/home/dlu/ros/pr2_navigation/pr2_navigation_super_config/params/default_move_base.yaml'
OLD_CONFIGURATION = '/home/dlu/ros/path_planning_metrics/path_planning_data/old_parameters.yaml'
DWA_PARAMS = '/home/dlu/ros/pr2_navigation/pr2_navigation_super_config/params/dwa_planner.yaml'
TPR_PARAMS = '/home/dlu/ros/pr2_navigation/pr2_navigation_config/move_base/base_local_planner_params.yaml'

class Parameter:
    def __init__(self, name, ns=None):
        self.name = name
        self.value = None
        self.count = None
        self.link = None
        self.ns = ns

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
            if type(self.value)==bool:
                if i==0:
                    return self.min
                else:
                    return self.max

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

    def get_full_name(self):
        if self.ns is None:
            return self.name
        else:
            return '%s/%s'%(self.ns, self.name)

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
    try:
        bits = []
        if '/' in array[0]:
            for s in array:
                a = s.split('/')
                if len(a)>1:
                    bits.append(a[1:])
                else:
                    bits.append(a)
        else:
            bits = array

        last_words = [a[-1] for a in bits]

        if all_unique(last_words):
            return last_words

        i=2
        while is_valid(i, bits):
            b = [a[-i] for a in bits]
            if not all_unique(b):
                i+=1
            else:
                return ['%s_%s'%(a[-i], a[-1]) for a in bits]

        return ['_'.join(a) for a in bits]
    except Exception, e:
        rospy.logerr("Trouble getting param keys")
        print array
        raise e


class Parameterization:
    def __init__(self, algorithm_fn, scenario_fn, variables, constants, basedir='.', node_name='move_base_node'):
        self.basedir = basedir
        self.node_name = node_name
        self.fixed_params = {}
        self.available_params = {}
        self.key_params = []

        node_ns = '/%s'%node_name
        self.parse_algorithm_file(algorithm_fn, node_ns)
        self.parse_scenario_file(scenario_fn)

        self.load_config(CORE_COSTMAP, ns=node_ns)
        self.load_config(MOVE_BASE,    ns=node_ns)

        for array, is_constant in [(constants, True), (variables, False)]:
            for key, value in parse_args(array):
                p = self.match_parameter(key)

                self.key_params.append(p.name)
                if is_constant:
                    p.set_value(value)
                else:
                    p.set_count(value)

        self.parameterizations = [{}]
        for name, p in self.available_params.iteritems():
            self.multiply(p)

        self.namespaces = ['/nav_experiments', node_ns]

    def parse_algorithm_file(self, filename, node_ns):
        config = yaml.load( open(filename) )
        self.algorithm = config['algorithm']
        self.fixed_params['/nav_experiments/algorithm'] = config['algorithm']
        self.fixed_params['/nav_experiments/topics'] = config['topics']
        if 'global_planner' in config:
            self.fixed_params['%s/base_global_planner'%(node_ns)] = config['global_planner']

        self.set_local_planner(config['local_planner'], node_ns)
        
        if 'critics' in config:
            self.load_critics( config['critics'], node_ns)

        if 'fuerte' in config['algorithm']:
            self.load_config(OLD_CONFIGURATION, ns=node_ns)
            self.fixed_params['/move_base_node/local_costmap/footprint_padding'] = 0.015
        else:
            self.load_layers( config['global_layers'], True, node_ns)
            self.load_layers( config['local_layers'], False, node_ns)

        for param in config.get('parameters', []):
            name = param['name']
            p = Parameter(name, ns=node_ns)
            if 'link' in param:
                p.set_link(param['link'])
            else:
                p.set_range(param['default'], param.get('min', None), param.get('max', None))
            self.available_params[name] = p

    def parse_scenario_file(self, filename):
        self.scenario = Scenario(filename)
        for name, m in self.scenario.vars.iteritems():
            p = Parameter(name, ns='/nav_experiments/scenario_params')
            p.set_range(m['default'], m['min'], m['max'])
            self.available_params[name] = p


    def load_config(self, filename, ns=None):
        config = yaml.load( open(filename) )
        m = {}
        for k,v in config.iteritems():
            if ns is None:
                m[k] = v
            else:
                m["%s/%s"%(ns,k)] = v
        self.fixed_params.update(m)

    def match_parameter(self, key):
        if key in self.available_params:
            return self.available_params[key]

        for pname in self.available_params:
            if key in pname:
                key = pname
                return self.available_params[key]

        return self.available_params[key] # Will throw appropriate error        

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

    def set_local_planner(self, name, node_ns):
        self.fixed_params['%s/base_local_planner'%node_ns] = name

        if 'dwa' in name:
            self.load_config(DWA_PARAMS, ns='%s/DWAPlannerROS'% node_ns)
        elif 'TrajectoryPlanner' in name:
            self.load_config(TPR_PARAMS, ns='%s/TrajectoryPlannerROS' % node_ns)
            rospy.set_param('/%s/move_slow_and_clear/planner_namespace'%self.name, 'TrajectoryPlannerROS')

    def set_params(self, m):
        for ns in self.namespaces:
            if rospy.has_param(ns):
                rospy.delete_param(ns)
        self.set_params_(self.fixed_params)
        self.set_params_(m)
        self.scenario.parameterize(m)
        rospy.set_param('/nav_experiments/scenario', self.scenario.get_scenario())

    def set_params_(self, m):
        for k, v in m.iteritems():
            if k[0]!='/':
                p = self.available_params[k]
                k = p.get_full_name()
            if rospy.has_param(k):
                v2 = rospy.get_param(k)
                if type(v2)==dict:
                    v3 = {}
                    v3.update(v)
                    v3.update(v2)
                    v = v3

            rospy.set_param(k, v)

    def get_folder(self):
        base = '%s/%s'%(self.basedir, self.scenario.key)
        if len(self.key_params)==0:
            return base
        keys = param_keys( sorted( self.key_params) )
        return '%s/%s'%(base, '-'.join(keys) )

    def get_filename(self, m, i):
        if len(self.key_params)==0:
            return '%s-%03d.bag'%(self.algorithm, i)

        s = self.algorithm
        for key in sorted( self.key_params ):
            s += '-%s'% str(m[key])
        s += '-%03d.bag'%i
        return s

    def get_full_filename(self, m, i):
        return '%s/%s'%(self.get_folder(), self.get_filename(m, i))

    def __repr__(self):
        return '===============\n'.join( [repr(x) for x in self.parameterizations] )
        
    def to_string(self, p):
        s = []
        for k in self.key_params:
            s.append( '%s:%s'%(k, str(p[k])))
        return ' '.join(s)
        
    def load_critics(self, critics, ns):
        ns = '%s/DWAPlannerROS/critics'%ns
        self.fixed_params[ns] = []
        for critic in critics:
            if '::' in critic:
                i = critic.index('::')+2
                name = critic[i:]
                self.fixed_params[ns].append({'name': name, 'type': critic})
            else:
                self.fixed_params[ns].append(critic)

    def load_layers(self, layers, is_global, ns):
        ns = '%s/%s_costmap'%(ns, 'global' if is_global else 'local')
        self.fixed_params['%s/plugins' % ns] = []
        for layer in layers:
            if layer == 'obstacles':
                self.add_standard_obstacle_layer(ns, False)
            elif layer == 'voxels':
                self.add_standard_obstacle_layer(ns, True)
            else:
                self.add_layer(ns, layer)
        
    def add_layer(self, ns, layer_type, layer_name=None, extra=None):
        layers = self.fixed_params['%s/plugins' % ns]
        if layer_name is None:
            i1 = layer_type.find(':')
            i2 = layer_type.find('Layer', i1)
            layer_name = layer_type[i1+2:i2].lower()
        layers.append( {'name': layer_name, 'type': layer_type} )

        if extra is not None:
            self.fixed_params['%s/%s'%(ns, layer_name)] = extra

    def add_standard_obstacle_layer(self, ns, voxel_layer=True):
        if voxel_layer:
            name = 'costmap_2d::VoxelLayer'
        else:
            name = 'costmap_2d::ObstacleLayer'

        self.add_layer(ns, name, 'obstacles', yaml.load(open('/home/dlu/ros/path_planning_metrics/path_planning_data/obstacles.yaml')))

