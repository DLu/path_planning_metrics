import roslib; roslib.load_manifest('path_planning_ruler')
import subprocess
import yaml
import rospy

def get_map(name, value):
    return '%s:=%s'%(name, value)

class MoveBaseInstance:
    def __init__(self, name='move_base_node', quiet=False):
        self.name = name
        self.process = None
        self.quiet = quiet
        self.void = open('/dev/null', 'w')

        pname = '/%s'%name
        if rospy.has_param(pname):
            rospy.delete_param(pname)


    def set_local_planner(self, name):
        rospy.set_param('/%s/base_local_planner'%self.name, name)

        if 'dwa' in name:
            self.load_config('/home/dlu/ros/pr2_navigation/pr2_navigation_super_config/params/dwa_planner.yaml', ns='DWAPlannerROS')
        elif 'TrajectoryPlanner' in name:
            self.load_config('/home/dlu/ros/pr2_navigation/pr2_navigation_config/move_base/base_local_planner_params.yaml', ns='TrajectoryPlannerROS')
            rospy.set_param('/%s/move_slow_and_clear/planner_namespace'%self.name, 'TrajectoryPlannerROS')
        
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

    def add_standard_obstacle_layer(self, is_global, voxel_layer=True):
        if voxel_layer:
            name = 'costmap_2d::VoxelLayer'
        else:
            name = 'costmap_2d::ObstacleLayer'

        self.add_layer(is_global, name, 'obstacles', yaml.load(open('/home/dlu/ros/path_planning_metrics/path_planning_data/obstacles.yaml')))


    def start(self):
        if self.quiet:
            self.process = subprocess.Popen(self.get_command(), stdout=self.void, stderr=self.void)
        else:
            self.process = subprocess.Popen(self.get_command())

    def shutdown(self):
        if self.process:
            self.process.terminate()

    def get_command(self):
        args = ['rosrun', 'move_base', 'move_base']
        args.append( get_map('_name', self.name) )
        args.append( get_map('odom', "/base_pose_ground_truth"))
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
        rospy.set_param('/nav_experiments/topics', config['topics'])

        self.load_config('/home/dlu/ros/path_planning_metrics/path_planning_data/core_costmap.yaml')

        self.load_config('/home/dlu/ros/pr2_navigation/pr2_navigation_super_config/params/default_move_base.yaml')

        if config['algorithm'] == 'fuerte':
            self.load_config('/home/dlu/ros/path_planning_metrics/path_planning_data/old_parameters.yaml')
            rospy.set_param('/move_base_node/local_costmap/footprint_padding', 0.015)
            return {}

        self.set_local_planner(config['local_planner'])

        self.load_layers( config['global_layers'], True)
        self.load_layers( config['local_layers'], False)

        values = {}
        for param in config.get('parameters', []):
            name = param['name']
            if name in parameterization:
                (N, i) = parameterization[name]
                spread = param['max'] - param['min']
                value = param['min'] + spread * i / N
                values[name] = value
            elif 'link' in param:
                link_name = param['link']
                fn = '/%s/%s'%(self.name, link_name)
                value = rospy.get_param(fn)
            else:
                value = param['default']

            fn = '/%s/%s'%(self.name, name)
            rospy.set_param(fn, value)
        return values

    def load_layers(self, layers, is_global):
        for layer in layers:
            if layer == 'obstacles':
                self.add_standard_obstacle_layer(is_global, False)
            elif layer == 'voxels':
                self.add_standard_obstacle_layer(is_global, True)
            else:
                self.add_layer(is_global, layer)


import sys
if __name__=='__main__':
    m = MoveBaseInstance()
    m.configure(sys.argv[1])

    m.start()
    raw_input("Press Enter")

    m.shutdown()
