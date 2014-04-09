#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_simulation')
from path_planning_simulation import *
import rospy
import gazebo_msgs.msg
from path_planning_analysis.translator import *
from people_msgs.msg import People, Person
from tf import Transformer
from std_srvs.srv import *

def get_position(moves, t):
    for i in range(len(moves)-1):
        next = moves[i+1]
        t2 = next['t']
        if t < t2:
            move = moves[i]
            p1 = move['pos']
            p2 = next['pos']
            t1 = move['t']
            pct = (t - t1)/(t2 - t1)
            p = [a+pct*(b-a) for a,b in zip(p1,p2)]
            return p
    if t > t2 + 1:
        return None
    return moves[-1]['pos']
    
class Animator:
    def __init__(self):
        rospy.init_node('animator')
        self.g = GazeboHelper(True)
        self.s = rospy.Service('/animator/reset', Empty, self.restart)
        self.objects = {}
        self.name_map = {}
        self.t = rospy.Time.now()
        self.pause = rospy.Duration(0)

    def restart(self, req):
        self.objects = rospy.get_param('/nav_experiments/scenario/objects', {})
        self.name_map = rospy.get_param('/nav_experiments/spawn_names', {})
        self.t = rospy.Time.now()
        self.pause = rospy.Duration(0)
        return EmptyResponse()

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            ellapsed = (rospy.Time.now() - self.t - self.pause).to_sec()
            for name, obj in self.objects.iteritems():
                if 'movement' in obj:
                    p = get_position(obj['movement'], ellapsed)
                    if p is None:
                        continue
                    pose = get_pose(p[0], p[1], p[2])                   
                    self.g.set_state( self.name_map[name], pose)
            r.sleep()

if __name__=='__main__':
    a = Animator()
    a.spin()
    
