import roslib; roslib.load_manifest('path_planning_analysis')
import rospy
import numpy
import scipy.spatial
from geometry_msgs.msg import Pose, TransformStamped, Polygon, Point32
from tf import Transformer
from tf.transformations import quaternion_from_euler
from math import sqrt

def distance(x0, y0, x1, y1):
    return sqrt(pow(x1-x0,2)+pow(y1-y0,2))

def distance_to_line(pX, pY, x0, y0, x1, y1):
    A = pX - x0
    B = pY - y0
    C = x1 - x0
    D = y1 - y0

    dot = A * C + B * D
    len_sq = C * C + D * D
    param = dot / len_sq

    if param < 0:
        xx = x0
        yy = y0
    elif param > 1:
        xx = x1
        yy = y1
    else:
        xx = x0 + param * C
        yy = y0 + param * D

    return distance(pX, pY, xx, yy)


def min_distance(px, py, polygon):
    min_dist = float("inf")
    for i, pt in enumerate(polygon.points):
        # check the distance from the point to the first vertex
        vertex_dist = distance(px, py, pt.x, pt.y)

        # check the distance from the point to the edge
        if i < len(polygon.points)-1:
            nextpt = polygon.points[i+1]
        else:
            nextpt = polygon.points[0]

        edge_dist = distance_to_line(px, py, pt.x, pt.y, nextpt.x, nextpt.y)

        min_dist = min(min_dist, min(vertex_dist, edge_dist))
    return min_dist

def get_convex_hull(vecs):
    vecs = numpy.array(vecs)
    hull = scipy.spatial.Delaunay(vecs).convex_hull
    ps = set()
    for x, y in hull:
        ps.add(x)
        ps.add(y)
    ps = numpy.array(list(ps))
    center = vecs[ps].mean(axis=0)
    A = vecs[ps] - center
    h = vecs[ps[numpy.argsort(numpy.arctan2(A[:,1], A[:,0]))]]
    return h

POM = [1, -1]
F = ' ab'

def get_vertices(pose, size, shape_type):
    tf = Transformer(True, rospy.Duration(10.0))
    m = TransformStamped()
    m.header.frame_id = 'object'
    m.transform.rotation.w = 1

    keys = []
    if shape_type=='box':
        for mx in POM:
            for my in POM:
                for mz in POM:
                    key = 'frame%s%s%s'%(F[mx], F[my], F[mz])
                    m.child_frame_id = key
                    keys.append(key)
                    m.transform.translation.x = size[0]/2.0*mx
                    m.transform.translation.y = size[1]/2.0*my
                    m.transform.translation.z = size[2]/2.0*mz  
                    tf.setTransform(m)
    else:
        return None

    m.transform.translation.x = pose.position.x
    m.transform.translation.y = pose.position.y
    m.transform.translation.z = pose.position.z
    m.transform.rotation.x = pose.orientation.x
    m.transform.rotation.y = pose.orientation.y
    m.transform.rotation.z = pose.orientation.z
    m.transform.rotation.w = pose.orientation.w
    m.header.frame_id = 'map'
    m.child_frame_id = 'object'
    tf.setTransform(m)
    
    vertices = []
    for key in keys:
        (off,rpy) = tf.lookupTransform('map', key, rospy.Time(0))
        vertices.append(off)
    return vertices

def get_polygon(pose, size, shape_type):
    vertices = get_vertices(pose, size, shape_type)
    projected = [(a[0], a[1]) for a in vertices]
    points = get_convex_hull(projected)
    p = Polygon()
    for x,y in points:
        p.points.append(Point32(x,y,0)) 
    return p

class ObjectField:
    def __init__(self, scenario, simulation_states, t0):
        self.static_objects = {}
        self.dynamic_objects = {}

        for name, m in scenario.iteritems():
            t = m.get('type', 'box')
            size = m.get('size', [1,1,1])
            obj = {'type': t, 'size': size}

            if 'movement' not in m:
                xyz = m.get('xyz', [0,0,0])
                rpy = m.get('rpy', [0,0,0])
                p = Pose()
                p.position.x = xyz[0]
                p.position.y = xyz[1]
                p.position.z = xyz[2]
                q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
                obj['pose'] = p
                get_polygon(p, size, t)
                self.static_objects[name] = obj
            else:
                poses = []
                for t, state in simulation_states: 
                    if not name in state.name:
                        continue
                    i = state.name.index(name)
                    poses.append((t-t0, state.pose[i]))
                obj['poses'] = sorted(poses)
                obj['polygons'] = [None] * len(poses)
                self.dynamic_objects[name] = obj

    def get_polygons(self, t0):
        polygons = {}
        for name, obj in self.static_objects.iteritems():
            if 'polygon' not in obj:
                obj['polygon'] = get_polygon(obj['pose'], obj['size'], obj['type'])
            polygons[name] = obj['polygon']
        
        for name, obj in self.dynamic_objects.iteritems():
            i0 = None
            for i, (t, pose) in enumerate(obj['poses']):
                if i==0 or t < t0:
                    continue
                dt0 = abs(t-t0)
                dt1 = abs(obj['poses'][i-1][0]-t0)
                if dt0 < dt1:
                    i0 = i
                else:
                    i0 = i - 1
                break
            if obj['polygons'][i0] is None:
                obj['polygons'][i0] = get_polygon(obj['poses'][i0][1], obj['size'], obj['type'])

            polygons[name] = obj['polygons'][i0]
        return polygons

    def get_nearest_polygon_distance(self, px, py, t):
        polygons = self.get_polygons(t)
        min_dist = float('inf')
        for name, polygon in polygons.iteritems():
            d = min_distance(px, py, polygon)
            min_dist = min(min_dist, d) 
        return min_dist
