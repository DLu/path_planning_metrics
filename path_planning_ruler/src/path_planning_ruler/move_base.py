import roslib; roslib.load_manifest('path_planning_ruler')
import subprocess
import yaml
import rospy

def get_map(name, value):
    return '_%s:=%s'%(name, value)

class MoveBaseInstance:
    def __init__(self, name='move_base_node'):
        self.name = name
        self.process = None
        self.load_config('/home/dlu/ros/pr2_navigation/pr2_navigation_super_config/params/default_move_base.yaml')
        self.load_config('/home/dlu/ros/path_planning_metrics/path_planning_data/core_costmap.yaml')
        

    def set_local_planner(self, name):
        rospy.set_param('/%s/base_local_planner'%self.name, name)

        if 'dwa' in name:
            self.load_config('/home/dlu/ros/pr2_navigation/pr2_navigation_super_config/params/dwa_planner.yaml', ns='DWAPlannerROS')
        elif 'TrajectoryPlanner' in name:
            self.load_config('/home/dlu/ros/pr2_navigation/pr2_navigation_config/move_base/base_local_planner_params.yaml', ns='TrajectoryPlannerROS')
        
    def add_layer(self, is_global, layer_type, layer_name=None, extra=None):
        ns = '/%s/%s_costmap'%(self.name, 'global' if is_global else 'local')
        layers = rospy.get_param('%s/plugins'%ns, [])
        if layer_name is None:
            i1 = layer_type.find(':')
            i2 = layer_type.find('Layer', i1)
            layer_name = layer_type[i1+2:i2].lower()
        layers.append( {'name': layer_name, 'type': layer_type} )
        rospy.set_param('%s/plugins'%ns, layers)
        if extra is not None:
            rospy.set_param('%s/%s'%(ns, layer_name), extra)

    def add_standard_obstacle_layer(self, is_global):
        self.add_layer(is_global, "costmap_2d::VoxelLayer", 'obstacles', yaml.load(open('/home/dlu/ros/path_planning_metrics/path_planning_data/obstacles.yaml')))


    def start(self):
        self.process = subprocess.Popen(self.get_command())

    def shutdown(self):
        if self.process:
            self.process.terminate()

    def get_command(self):
        args = ['rosrun', 'move_base', 'move_base']
        args.append( get_map('name', self.name) )
        args.append( get_map('odom', "base_odometry/odom"))
        args.append( get_map("cmd_vel", "navigation/cmd_vel"))

        return args

    def load_config(self, filename, ns=None):
        config = yaml.load( open(filename) )
        namespace = '/%s'%self.name
        if ns is not None:
            namespace += '/%s'%ns
        for k,v in config.iteritems():
            rospy.set_param("%s/%s"%(namespace,k), v)

    def configure(self, filename, parameterization={}):
        config = yaml.load( open(filename) )
        rospy.set_param('/nav_experiments/algorithm', config['algorithm'])
        self.set_local_planner(config['local_planner'])

        self.load_layers( config['global_layers'], True)
        self.load_layers( config['local_layers'], False)

        values = {}
        for param in config.get('parameters', []):
            name = param['name']
            if name in parameterization:
                (N, i) = parameterization[name]
                spread = param['max'] - param['min']
                value = param['min'] + spread * i / (N - 1)
                values[name] = value
            else:
                value = param['default']
            rospy.set_param(name, value)
        return values

    def load_layers(self, layers, is_global):
        for layer in layers:
            if layer == 'obstacles':
                self.add_standard_obstacle_layer(is_global)
            else:
                self.add_layer(is_global, layer)


if __name__=='__main__':
    m = MoveBaseInstance()
    m.start()
    m.set_local_planner('dwa_local_planner/DWAPlannerROS')
    m.add_layer(True, "costmap_2d::StaticLayer")
    m.add_standard_obstacle_layer(True)
    m.add_standard_obstacle_layer(False)
    raw_input("Press Enter")

    m.shutdown()
