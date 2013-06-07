#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_simulation')
import rospy
import gazebo_msgs.msg
import std_msgs.msg 
class CollisionWatcher:
    def __init__(self):
        rospy.init_node('collision_watcher')
        self.collisions = set()
        self.sub = rospy.Subscriber('/base_bumper', gazebo_msgs.msg.ContactsState, self.get_contact)
        self.pub = rospy.Publisher('/collisions', std_msgs.msg.String)


    def get_contact(self, msg):
        if len(msg.states)==0:
            return
        for state in msg.states:
            self.collisions.add( (state.collision1_name, state.collision2_name))

    def spin(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            latest = self.collisions
            self.collisions = set()
            for name1, name2 in latest:
                s = std_msgs.msg.String()
                s.data = "%s <==> %s"%(name1, name2)
                self.pub.publish(s)
            r.sleep()

c = CollisionWatcher()
c.spin()
