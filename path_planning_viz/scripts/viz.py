#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_viz')
import rosbag
import rospy
import collections
import sys
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

def get_square(res, i, j, pos, R=1):
    pts = []
    x = pos.x + res*i
    y = pos.y + res*j
    z = pos.z

    pts.append( Point( x, y, z ) )
    pts.append( Point( x, y+res*R, z ) )
    pts.append( Point( x+res*R, y, z ) )
    pts.append( Point( x+res*R, y+res*R, z ) )
    pts.append( Point( x, y+res*R, z ) )
    pts.append( Point( x+res*R, y, z ) )

    return pts

def getPose(p):
    pose = PoseStamped()
    if p:
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/map'
        pose.pose.position.x = p.x
        pose.pose.position.y = p.y
        quat = quaternion_from_euler(0,0,p.theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

    return pose
    
def update_map(grid, update):
    xgrid = OccupancyGrid()
    xgrid.header.stamp = update.header.stamp
    xgrid.header.frame_id = update.header.frame_id
    xgrid.info = grid.info
    xgrid.data = grid.data[:]

    i = 0
    for yi in range(update.height):
        for xi in range(update.width):
            index = (update.y+yi)*update.width + xi + update.x
            xgrid.data[index] = update.data[i]
            i+=1
    return xgrid
    
def get_costmap_marker(grid, ns, local=False):
    print len(grid.data), ns, local, sum(grid.data)
    marker = Marker()
    marker.header.frame_id = grid.header.frame_id
    marker.header.stamp = grid.header.stamp
    marker.ns = 'local' if local else 'global'
    marker.id = 1 if local else 0
    marker.type = Marker.TRIANGLE_LIST
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color = ColorRGBA(1.0, 0, 0, 1.0)
    pos = grid.info.origin.position

    for j in range(0, grid.info.height):
        for i in range(0, grid.info.width):
            index = j * grid.info.width + i
            value = grid.data[index]

            pts = get_square(grid.info.resolution, i, j, pos)
            marker.points += pts

            cv = 1.0 - value / float(100)
            if value == 0:
                color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            elif local:
                color = ColorRGBA(cv, 0, 0, 1.0)
            else:
                color = ColorRGBA(0, cv, 0, 1.0)

            marker.colors += [color] * len(pts)

    return marker

if len(sys.argv)>1:
    fn = sys.argv[1]
else:
    fn = '/home/dlu/Desktop/path_data/core/09_personobstacle-hydro_collect-000.bag'
bag = rosbag.Bag(fn)

#costmaps
#global path
#local path

start = None
goal = None
poses = []
updates = collections.defaultdict(list)
maps = collections.defaultdict(dict)
tmp = {}

for topic, msg, t in bag.read_messages():
    if topic == '/start':
        start = msg
    elif topic == '/goal':
        goal = msg
    elif topic == '/robot_pose':
        poses.append(msg)
    elif 'costmap_update' in topic:
        print sum(msg.data)
        updates[topic].append((len(poses), msg))
    elif '/costmap' in topic:
        msg.data = list(msg.data)
        print sum(msg.data)
        maps[topic][0] = msg
        tmp[topic] = msg

for topic in maps:
    for t, update in updates[topic + "_updates"]:
        if t not in maps[topic]:
            maps[topic][t] = tmp[topic]
        maps[topic][t] = update_map(maps[topic][t], update)
        tmp[topic] = maps[topic][t]
    

t = 0
rospy.init_node('s')
spub = rospy.Publisher('/start', PoseStamped)
gpub = rospy.Publisher('/goal', PoseStamped)
rpub = rospy.Publisher('/robot', PoseStamped)
mpub = rospy.Publisher('/visualization_marker', Marker)


while t < len(poses):
    print "%d/%d"%(t,len(poses))
    spub.publish( getPose(start) )
    gpub.publish( getPose(goal) )
    rpub.publish( getPose( poses[t] ) )
    
    for topic in maps:
        if not t in maps[topic]:
            continue
        grid = maps[topic][t]
        mpub.publish( get_costmap_marker(grid, topic, 'local' in topic) )
    t+=1

    x = raw_input("?")
    if 'x' in x:
        break

    
