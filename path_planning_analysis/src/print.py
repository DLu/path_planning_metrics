#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
import rosbag

if __name__=='__main__':
    topics = []
    bags = []
    for arg in sys.argv[1:]:
        if '.bag' in arg:
            bags.append(arg)
        else:
            topics.append(arg)

    for bag_fn in bags:
        if len(bags)>1:
            print bag_fn
        bag = rosbag.Bag(bag_fn, 'r')
        for topic, msg, t in bag.read_messages(topics=topics):
            print msg
