import roslib; roslib.load_manifest('path_planning_analysis')
import sys
import rosbag
import rospy
import collections
import pylab
import yaml
from math import sin, cos, sqrt, pi, atan2
from path_planning_analysis.object_field import ObjectField
from geometry_msgs.msg import Polygon as RosPolygon, Point32
PIx2 = pi * 2

def smooth(x, window=2):
    N = window * 2 + 1
    result = []
    warr = []

    for i, xi in enumerate(x):
        warr.append(xi)
        while len(warr) > N:
            warr = warr[1:]

        if len(warr) >= window + 1:
            result.append(sum(warr)/len(warr))
    for i in range(window):
        warr = warr[1:]
        result.append(sum(warr)/len(warr))

    return result

def plot_path(ax, path):
    x = [p.pose.position.x for p in path.poses]
    y = [p.pose.position.y for p in path.poses]
    ax.plot(x,y)

def to_triple(pose):
    return [pose.x, pose.y, pose.theta]

class RobotPath:
    def __init__(self, filename):
        self.filename = filename
        self.t0 = None
        self.poses = []
        self.obstacles = []
        self.local_times = []
        self.global_times = []
        self.other = collections.defaultdict(list)
        try:
            bag = rosbag.Bag(filename, 'r')
            for topic, msg, t in bag.read_messages():
                if topic=='/robot_pose':
                    if self.t0 is None:
                        self.t0 = t
                        self.poses.append((rospy.Duration(0),msg))
                    else:
                        ellapsed = t-self.t0
                        last = ellapsed-self.poses[-1][0]
                        if last.to_sec()>.001:
                            self.poses.append((t-self.t0,msg))
                elif topic=='/simulation_state':
                    self.obstacles.append((t,msg))
                elif 'update_time' in topic:
                    if 'global' in topic:
                        self.global_times.append(msg.data)
                    else:
                        self.local_times.append(msg.data)
                else:
                    self.other[topic].append((t,msg))
            bag.close()
            if len(self.poses)==0:
                self.valid = False
            else:
                self.valid = True

            scenario_objects = self.get_scenario().get('objects', {})
            self.object_field = ObjectField(scenario_objects, self.obstacles, self.t0)
        except:
            sys.stderr.write("Cannot read bag file %s\n" % filename)
            self.valid = False

    def get_deltas(self, start_off=0, end_off=2):
        deltas = []
        for i in range(len(self.poses)):
            si = max(0, i + start_off)
            ei = min(len(self.poses)-1, i + end_off)
            t1, p1 = self.poses[si]
            t2, p2 = self.poses[ei]
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            dz = p2.theta - p1.theta
            t = (t2-t1).to_sec()
            if t==0:
                t = 1
            deltas.append( (dx/t, dy/t, dz/t) )
        return deltas

    def get_velocity(self, start_off=0, end_off=2):
        deltas = self.get_deltas(start_off, end_off)
        vels = []
        for dx,dy,dz in deltas:
            vels.append( ( atan2(dy,dx), sqrt(dx*dx+dy*dy)))
        return vels

    def get_distances_to_objects(self):
        distances = []
        for t, pose in self.poses:
            dist = self.object_field.get_nearest_polygon_distance(pose.x, pose.y, t)
            distances.append( dist )    
        return distances

    def get_distances_to_people(self):
        distances = []
        for t, pose in self.poses:
            dist = self.object_field.get_nearest_polygon_distance(pose.x, pose.y, t, person=True)
            distances.append( dist )    
        return distances

    def get_distances_to_objects_with_polygon(self, width, length, angle_mod):
        distances = []
        for t, pose in self.poses:
            theta = pose.theta + angle_mod
            dx = cos(theta) * length
            dy = sin(theta) * length
            bx = cos(theta + pi / 2) * width / 2
            by = sin(theta + pi / 2) * width / 2

            p = RosPolygon()
            p.points.append(Point32(pose.x + bx,      pose.y + by,      0.0))
            p.points.append(Point32(pose.x - bx,      pose.y - by,      0.0))
            p.points.append(Point32(pose.x - bx + dx, pose.y - by + dy, 0.0))
            p.points.append(Point32(pose.x + bx + dx, pose.y + by + dy, 0.0))

            dist = self.object_field.get_nearest_distance_in_polygon(pose.x, pose.y, t, p)
            distances.append( dist )
        return distances

    def plot(self):
        ax = pylab.axes()
        pylab.axis('equal')
        self.plot(ax)
        pylab.show()
        
    def plot_one(self, ax):
        x =[]
        y =[]
        for t, pose in self.poses:
            x.append(pose.x)
            y.append(pose.y)
        ax.plot(x,y)
        for t, pose in self.poses:
            theta = pose.theta #+ pi
            dx = cos(theta) / 500
            dy = sin(theta) / 500
            ax.arrow(pose.x, pose.y, dx, dy, head_width=.025, head_length=.05)    

    def plot_progress(self):
        ax = pylab.axes()
        #pylab.axis('equal')

        x =[]
        y =[]
        z =[]
        ts=[]
        for t, pose in self.poses:
            ts.append(t.to_sec())
            x.append(pose.x)
            y.append(pose.y)
            z.append(pose.theta)
        ax.plot(ts,x)
        ax.plot(ts,y)
        ax.plot(ts,z)
        pylab.show()

    def plot_global(self, ax):
        for t, path in self.other['/move_base_node/NavfnROS/plan']:
            plot_path(ax, path)

    def plot_local(self, ax):
        for t, path in self.other['/move_base_node/DWAPlannerROS/local_plan']:
            plot_path(ax, path)

    def collisions(self):
        return 1.0 if len(self.other['/collisions'])>0 else 0.0

    def polygon_distances_helper(self, angle, width=1, MAX=100):
        return self.get_distances_to_objects_with_polygon(width, MAX, angle)

    def front_distances(self):
        return self.polygon_distances_helper(0.0)

    def left_distances(self):
        return self.polygon_distances_helper(pi/2)

    def right_distances(self):
        return self.polygon_distances_helper(-pi/2)

    def get_curvatures(self, delta=5, precision=2):
        backward = self.get_deltas(-1*delta, 0)
        forward = self.get_deltas(0, delta)

        sums = []
        for bb, ff in zip(backward, forward):
            vv = [round(b,precision) + round(f,precision) for b,f in zip(bb,ff)]
            aa = [round(f,precision) - round(b,precision) for b,f in zip(bb,ff)]
            xp = vv[0]
            yp = vv[1]
            if xp == 0.0 and yp==0.0:
                sums.append(0.0)
                continue
            xpp = aa[0]
            ypp = aa[1]
            k = abs(xp * ypp - xpp * yp) / pow(xp*xp + yp*yp, 3.0/2.0)
            
            sums.append(min(100000000,k))
        return sums

    def get_scenario_name(self):
        return self.filename.split('-')[0]

    def get_algorithm(self):
        return self.filename.split('-')[1]

    def get_scenario(self):
        #TODO dynamically code this
        return yaml.load(open('/home/dlu/ros/path_planning_metrics/path_planning_data/scenarios/%s.yaml'%self.get_scenario_name()))

    def get_data(self):
        vels = self.get_velocity()

        return {
            't': [t for t,x in self.poses], 
            'poses': [to_triple(x) for t,x in self.poses],
            'object_distances': self.get_distances_to_objects(),
            'people_distances': self.get_distances_to_people(),
            'front_distances': self.front_distances(),
            'left_distances': self.left_distances(),
            'right_distances': self.right_distances(),
            'curvatures': self.get_curvatures(),
            'collisions': self.collisions(),
            'headings': [heading for heading, magnitude in vels],
            'speeds': [magnitude for heading, magnitude in vels],
            'start_pose': to_triple(self.other['/start'][0][1]),
            'goal_pose': to_triple(self.other['/goal'][0][1]),
            'global_times': self.global_times,
            'local_times': self.local_times
        }

    def stats(self):
        return self.completed, self.rotate_efficiency, self.translate_efficiency, self.time, self.collisions, self.minimum_distance_to_obstacle, self.average_distance_to_obstacle, self.face_direction_of_travel, self.curvature, self.front_distance, self.left_distance, self.right_distance

        
