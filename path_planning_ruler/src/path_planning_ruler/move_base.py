import roslib; roslib.load_manifest('path_planning_ruler')
import subprocess
import yaml
import rospy

def get_map(name, value):
    return '_%s:=%s'%(name, value)

class MoveBaseInstance:
    def __init__(self, name='move_base_node'):
        self.name = name
        self.load_config('/home/dlu/ros/pr2_navigation/pr2_navigation_super_config/params/default_move_base.yaml')
        self.start()

    def start(self):
        self.process = subprocess.Popen(self.get_command())

    def shutdown(self):
        self.process.terminate()

    def get_command(self):
        args = ['rosrun', 'move_base', 'move_base']
        args.append( get_map('name', self.name) )
        args.append( get_map('odom', "base_odometry/odom"))
        args.append( get_map("cmd_vel", "navigation/cmd_vel"))

        return args

    def load_config(self, filename):
        config = yaml.load( open(filename) )
        for k,v in config.iteritems():
            rospy.set_param("/%s/%s"%(self.name,k), v)

if __name__=='__main__':
    m = MoveBaseInstance()

    raw_input("Press Enter")

    m.shutdown()
