import collections
import rosbag
import rospy, tf
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray

from tf.transformations import quaternion_from_euler
from std_msgs.msg import ColorRGBA
ALL_MAP_FRAMES = True

def update_map(grid, update):
    xgrid = OccupancyGrid()
    xgrid.header.stamp = update.header.stamp
    xgrid.header.frame_id = update.header.frame_id
    xgrid.info = grid.info
    xgrid.data = grid.data[:]

    i = 0
    for yi in range(update.height):
        for xi in range(update.width):
            index = (update.y+yi)*xgrid.info.width + xi + update.x
            xgrid.data[index] = update.data[i]
            i+=1
    return xgrid

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


    

    
def get_costmap_marker(grid, local=False, JUMP=1):
    marker = Marker()
    marker.header.frame_id = '/map' if ALL_MAP_FRAMES else grid.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = 'local' if local else 'global'
    marker.id = 1 if local else 0
    marker.type = Marker.TRIANGLE_LIST
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color = ColorRGBA(1.0, 0, 0, 1.0)
    pos = grid.info.origin.position

    for j in range(0, grid.info.height, JUMP):
        for i in range(0, grid.info.width, JUMP):

            index = j * grid.info.width + i 
            value = grid.data[index]
            for dj in range(JUMP):
                for di in range(JUMP):
                    index = (j+dj) * grid.info.width + i + di
                    value = max(value, grid.data[index])

            pts = get_square(grid.info.resolution, i, j, pos, JUMP)
            if value == 0:
                continue
            marker.points += pts

            cv = 1.0 - value / float(100)
            if value == 100:
                color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            elif value == 99:
                color = ColorRGBA(.2, .2, .2,1.0)
            elif local:
                color = ColorRGBA(cv, 0, 0, 1.0)
            else:
                color = ColorRGBA(0, cv, 0, 1.0)

            marker.colors += [color] * len(pts)
    return marker




class CondensedBag:
    def __init__(self, filename, globalmap=False, localmap=False):
        bag = rosbag.Bag(filename)
        self.start = None
        self.goal = None
        self.poses = []
        self.maps = collections.defaultdict(dict)
        self.paths = collections.defaultdict(dict)
        self.obstacle = None
        self.footprint = None
        updates = collections.defaultdict(list)
        tmp = {}

        for topic, msg, t in bag.read_messages():
            if topic == '/start':
                self.start = msg
            elif topic == '/goal':
                self.goal = msg
            elif topic == '/robot_pose':
                self.poses.append(msg)
            elif 'costmap_update' in topic:
                if not globalmap and 'global' in topic:
                    continue
                if not localmap and 'local' in topic:
                    continue
                print topic
                updates[topic].append((len(self.poses), msg))
            elif '/costmap' in topic:
                if not globalmap and 'global' in topic:
                    continue
                if not localmap and 'local' in topic:
                    continue
                msg.data = list(msg.data)
                self.maps[topic][len(self.poses)] = msg
                if topic not in tmp:
                    tmp[topic] = msg
            elif 'local_plan' in topic:
                self.paths['/local_plan'][len(self.poses)] = msg
            elif '/plan' in topic:
                self.paths['/global_plan'][len(self.poses)] = msg

        for topic in self.maps:
            for t, update in updates[topic + "_updates"]:
                if t not in self.maps[topic]:
                    self.maps[topic][t] = tmp[topic]
                self.maps[topic][t] = update_map(self.maps[topic][t], update)
                tmp[topic] = self.maps[topic][t]

class Republisher:
    def __init__(self, pathtopics, publish_grid = False):
        self.spub = rospy.Publisher('/start', PoseStamped)
        self.gpub = rospy.Publisher('/goal', PoseStamped)
        self.rpub = rospy.Publisher('/robot', PoseStamped)
        self.mpub = rospy.Publisher('/visualization_marker', Marker)
        self.fpub = rospy.Publisher('/footprint', PolygonStamped)
        self.xpub = rospy.Publisher('/polygon', PolygonStamped)

        if publish_grid:
            self.opub = rospy.Publisher('/costmap', OccupancyGrid)
        else:
            self.opub = None

        self.jump = 1

        self.transforms = {}
        self.br = tf.TransformBroadcaster()
        self.ppub = {}
        for topic in pathtopics:
            pub = rospy.Publisher(topic, Path)
            self.ppub[topic] = pub

        self.timer = rospy.Timer(rospy.Duration(.1), self.my_callback)

    def my_callback(self, event):
        for name, (parent, child, pos, orientation) in self.transforms.iteritems():
            self.br.sendTransform(pos,
                             tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2]),
                             rospy.Time.now(), child, parent)

    def publish(self, bag, t):
        print "%d/%d"%(t,len(bag.poses))
        self.spub.publish( getPose(bag.start) )
        self.gpub.publish( getPose(bag.goal) )
        if bag.obstacle:
            bag.obstacle.header.stamp = rospy.Time.now()
            self.xpub.publish(bag.obstacle)
        if bag.footprint:
            bag.footprint.header.stamp = rospy.Time.now()
            self.fpub.publish(bag.footprint)

        pose = bag.poses[t]
        self.rpub.publish( getPose( pose ) )
        
        for topic in bag.maps:
            if not t in bag.maps[topic]:
                continue
            grid = bag.maps[topic][t]
            self.mpub.publish( get_costmap_marker(grid, 'local' in topic, self.jump) )

            if self.opub and 'local' in topic:
                self.opub.publish(grid)

        for topic in bag.paths:
            if not t in bag.paths[topic]:
                continue
            path = bag.paths[topic][t]
            path.header.stamp =rospy.Time.now()
            path.header.frame_id = '/map'
            self.ppub[topic].publish( path )

        self.transforms['to_base'] = ('/map', '/odom_combined', (pose.x, pose.y, 0), (0,0,pose.theta))

