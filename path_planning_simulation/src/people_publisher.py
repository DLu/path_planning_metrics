#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_simulation')
import rospy
import gazebo_msgs.msg
from path_planning_analysis.translator import *
from people_msgs.msg import People, Person

class PeoplePublisher:
    def __init__(self):
        rospy.init_node('people_publisher')
        self.sub = rospy.Subscriber('/simulation_state', gazebo_msgs.msg.ModelStates, self.model_state_cb)
        self.pub = rospy.Publisher('/people', People)

    def model_state_cb(self, msg):
        self.names = set( rospy.get_param('/nav_experiments/people', []) )
        objects = rospy.get_param('/nav_experiments/scenario/objects', {})

        people_list = People()
        people_list.header.stamp = rospy.Time.now()
        people_list.header.frame_id = '/map'
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if len(self.names)==len(people_list.people):
                break
            if name not in self.names:
                continue

            p = Person()
            p.name = name

            properties = objects[name]
            if 'movement' in properties:
                pose = get_pose_from_scenario(name, properties)
            p.position = pose.position

            p.velocity.x  = twist.linear.x
            p.velocity.y  = twist.linear.y
            p.velocity.z  = twist.linear.z
            p.reliability = 1.0
            people_list.people.append(p)
        self.pub.publish(people_list)

if __name__=='__main__':
    pp = PeoplePublisher()
    rospy.spin()
    
