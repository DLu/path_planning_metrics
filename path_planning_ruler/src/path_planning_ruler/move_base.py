import roslib; roslib.load_manifest('path_planning_ruler')
import subprocess
import yaml
import rospy

def get_map(name, value):
    return '%s:=%s'%(name, value)

class MoveBaseInstance:
    def __init__(self, quiet=False):
        self.process = None
        self.quiet = quiet
        self.void = open('/dev/null', 'w')

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

