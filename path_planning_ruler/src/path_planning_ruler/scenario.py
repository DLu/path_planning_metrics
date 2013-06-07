import roslib; roslib.load_manifest('path_planning_ruler')
from path_planning_simulation import *
from path_planning_simulation.models import *
import rospy
import yaml
import os.path
from geometry_msgs.msg import Pose2D
import threading

def get_pose2d(scenario, key):
    pose = Pose2D()
    if key in scenario:
        value = scenario[key]
        if type(value)==type([]):
            pose.x = value[0]
            pose.y = value[1]
            pose.theta = value[2]
        else:
            print "Unknown type"
    return pose

def pose2d_to_pose(pose):
    return get_pose(pose.x, pose.y, pose.theta)

class ScenarioUpdater(threading.Thread):
    def __init__(self, scenario):
        threading.Thread.__init__(self)
        self.scenario = scenario
        self.running = False

    def run(self):
        self.running = True
        rate = rospy.Rate(5)
        while(self.running):
            rate.sleep()

    def stop(self):
        self.running = False


class Scenario:
    def __init__(self, filename):
        scenario = yaml.load( open(filename) )
        self.start = get_pose2d(scenario, 'start')
        self.goal = get_pose2d(scenario, 'goal')
        self.key = os.path.splitext( os.path.basename(filename) )[0]

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    def reset(self, gazebo):
        gazebo.set_state('pr2', pose2d_to_pose(self.start))
        rospy.sleep(1.0)

    def start_update_loop(self):
        self.updater = ScenarioUpdater(self)
        self.updater.start()

    def stop_update_loop(self):
        self.updater.stop()

