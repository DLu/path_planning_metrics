import roslib; roslib.load_manifest('path_planning_analysis')
import rospy
import numpy
import scipy.spatial
from path_planning_analysis.translator import *
from geometry_msgs.msg import Pose, TransformStamped, Point32
from geometry_msgs.msg import Polygon as RosPolygon
from tf import Transformer
from tf.transformations import quaternion_from_euler
from math import sqrt
import Polygon #https://github.com/jraedler/Polygon2/

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

def line_intersection_helper(a1,a2,b1,b2):
    denom =      (b2.y-b1.y)*(a2.x-a1.x) - (b2.x-b1.x)*(a2.y-a1.y)
    if denom==0:
        return None
    numerator1 = (b2.x-b1.x)*(a1.y-b1.y) - (b2.y-b1.y)*(a1.x-b1.x)
    numerator2 = (a2.x-a1.x)*(a1.y-b1.y) - (a2.y-a1.y)*(a1.x-b1.x)
    return numerator1/denom, numerator2/denom

def line_intersection_helper2(a1,a2,f):
    x = a1.x + f * (a2.x - a1.x)
    y = a1.y + f * (a2.y - a1.y)
    return x,y

def line_intersection(a1,a2,b1,b2, segments=False):
    intersection = line_intersection_helper(a1,a2,b1,b2)
    if not intersection:
        return None

    ua, ub = intersection
    if segments:
        if ua < 0 or ua > 1 or ub < 0 or ub > 1:
            return None

    return line_intersection_helper2(a1, a2, ua)

def point_in_poly(point, poly):
    inside = False

    for i, pt1 in enumerate(poly.points):
        pt2 = poly.points[i-1]
        if point.y > min(pt1.y,pt2.y):
            if point.y <= max(pt1.y,pt2.y):
                if point.x <= max(pt1.x,pt2.x):
                    if pt1.y != pt2.y:
                        xints = (point.y-pt1.y)*(pt2.x-pt1.x)/(pt2.y-pt1.y)+pt1.x
                    if pt1.x == pt2.x or point.x <= xints:
                        inside = not inside
    return inside

def poly_to_ros(p):
    newpoly = RosPolygon()
    newpoly.points = [Point32(x,y,0) for x,y in p]
    return newpoly

def ros_to_poly(p):
    return Polygon.Polygon( [(pt.x,pt.y) for pt in p.points]) 

def polygon_intersection(poly1, poly2):
    q = ros_to_poly(poly1)
    t = ros_to_poly(poly2)
    intersection = q & t
    if len(intersection)==0:
        return None
    return poly_to_ros(intersection[0])
            

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
    p = RosPolygon()
    for x,y in points:
        p.points.append(Point32(float(x),float(y),0)) 
    return p

class ObjectField:
    def __init__(self, scenario, name_map, simulation_states, t0):        
        self.static_objects = {}
        self.dynamic_objects = {}

        for name, obj in scenario.iteritems():
            t = obj.type
            size = obj.size
            new_obj = {'type': t, 'size': size, 'person': obj.is_person}
            oname = name_map.get(name, name)
            if obj.movement is None:
                p = get_pose_from_scenario(name, obj)
                new_obj['pose'] = p
                get_polygon(p, size, t)
                self.static_objects[name] = new_obj
            else:
                poses = []
                for t, state in simulation_states: 
                    if not oname in state.name:
                        continue
                    p = get_pose_from_state(oname, state)
                    poses.append((t-t0, p))
                new_obj['poses'] = sorted(poses)
                new_obj['polygons'] = [None] * len(poses)
                self.dynamic_objects[name] = new_obj

    def get_polygons(self, t0, person=False):
        polygons = {}
        for name, obj in self.static_objects.iteritems():
            if person and not obj['person']:
                continue
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

    def nearest_distance_helper(self, px, py, polygons):
        min_dist = float('inf')
        for polygon in polygons:
            d = min_distance(px, py, polygon)
            min_dist = min(min_dist, d) 
        return min_dist

    def get_nearest_polygon_distance(self, px, py, t, person=False):
        polygons = self.get_polygons(t, person)
        return self.nearest_distance_helper(px, py, polygons.values())

    def get_nearest_distance_in_polygon(self, px, py, t, mask, person=False):
        masked_polygons = []
        for name, poly in self.get_polygons(t, person).iteritems():
            new_poly = polygon_intersection(mask, poly)
            if new_poly:
                masked_polygons.append(new_poly)
        return self.nearest_distance_helper(px, py, masked_polygons)
