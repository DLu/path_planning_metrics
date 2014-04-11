#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_simulation')
import rospy
import gazebo_msgs.msg
from path_planning_analysis.translator import *
from people_msgs.msg import PositionMeasurement, PositionMeasurementArray
from geometry_msgs.msg import TransformStamped
import tf

class PeoplePublisher:
    def __init__(self):
        rospy.init_node('people_publisher')
        self.sub = rospy.Subscriber('/simulation_state', gazebo_msgs.msg.ModelStates, self.model_state_cb)
        self.pub = rospy.Publisher('/people_tracker_measurements', PositionMeasurementArray)
        self.tf = tf.Transformer()

    def model_state_cb(self, msg):
        self.names = set( rospy.get_param('/nav_experiments/people', []) )
        objects = rospy.get_param('/nav_experiments/scenario/objects', {})

        people_list = PositionMeasurementArray()
        people_list.header.stamp = rospy.Time.now()
        people_list.header.frame_id = '/map'
        for name, pose, twist in zip(msg.name, msg.pose, msg.twist):
            if len(self.names)==len(people_list.people):
                break
            if name not in self.names:
                continue

            p = PositionMeasurement()
            oname = name[:-10]
            p.name = oname
            p.object_id = oname
            p.reliability = 1.0
            

            properties = objects[oname]
            if 'movement' not in properties:
                p.pos.x = properties['xyz'][0]
                p.pos.y = properties['xyz'][1]
                p.pos.z = properties['xyz'][2]
            else:
                trans = TransformStamped()
                trans.header.frame_id = '/map'
                trans.child_frame_id = '/start'
                trans.transform.translation.x = properties['xyz'][0]
                trans.transform.translation.y = properties['xyz'][1]
                trans.transform.translation.z = properties['xyz'][2]
                trans.transform.rotation.x = 0
                trans.transform.rotation.y = 0
                trans.transform.rotation.z = 0
                trans.transform.rotation.w = 1
                self.tf.setTransform(trans)
                trans.header.frame_id = '/start'
                trans.child_frame_id = '/pos'
                trans.transform.translation = pose.position
                self.tf.setTransform(trans)
                nt = self.tf.lookupTransform('/map', '/pos', rospy.Time(0))
                
                p.pos.x = nt[0][0]
                p.pos.y = nt[0][1]
                p.pos.z = nt[0][2]
            p.header.stamp = people_list.header.stamp
            p.header.frame_id = '/map'
            people_list.people.append(p)
        self.pub.publish(people_list)

if __name__=='__main__':
    pp = PeoplePublisher()
    rospy.spin()
    
