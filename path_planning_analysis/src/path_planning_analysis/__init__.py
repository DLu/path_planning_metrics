import roslib; roslib.load_manifest('path_planning_analysis')
import rosbag
import collections
import pylab
from math import sin, cos, sqrt, pi
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

def derivative(t, x):
    ds = []
        
    for i, (ti, xi) in enumerate(zip(t,x)):
        if i>0:
            ds.append((xi-x[i-1])/(ti-t[i-1]))
        else:   
            ds.append(0)
    return ds
        

def dist(p1, p2):
    return sqrt( pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) )

def a_dist(p1, p2):
    diff = p1.theta - p2.theta
    mod_diff = abs(diff % PIx2)
    return min(mod_diff, PIx2-mod_diff)

class RobotPath:
    def __init__(self, filename):
        bag = rosbag.Bag(filename, 'r')
        self.t0 = None
        self.poses = []
        self.other = collections.defaultdict(list)
        for topic, msg, t in bag.read_messages():
            if topic=='/robot_pose':
                if self.t0 is None:
                    self.t0 = t
                self.poses.append((t-self.t0,msg))
            else:
                self.other[topic].append((t,msg))
        bag.close()

    def get_displacement(self):
        ts = [] 
        ds = []
        prev = None
        for t, pose in self.poses:
            if prev is None:
                prev = pose
            
            ts.append(t.to_sec())
            ds.append(dist(pose, prev))
            prev = pose
        return ts, ds

    def get_velocity(self):
        ts, ds = self.get_displacement()
        vs = []
        
        for i, (ti, xi) in enumerate(zip(ts,ds)):
            if i>0:
                vs.append(xi/(ti-ts[i-1]))
            else:
                vs.append(0)
        return vs

    def plot(self):
        ax = pylab.axes()
        pylab.axis('equal')

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
            print theta
            ax.arrow(pose.x, pose.y, dx, dy, head_width=.025, head_length=.05)
        pylab.show()

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

    def translate_efficiency(self):
        ts, ds = self.get_displacement()
        D = sum(map(abs, ds))
        D0 = dist(self.poses[0][1], self.poses[-1][1])
        denom = 1 if D0 == 0 else D0
        return (D-D0)/denom
        
    def rotate_efficiency(self):
        p0 = None
        A = 0.0
        for t, pose in self.poses:
            if not p0:
                p0 = pose
            A += abs(a_dist(p0, pose))
            p0 = pose
        A0 = a_dist(self.poses[0][1], self.poses[01][1])
        denom =1 if A0 == 0 else A0
        return (A-A0)/denom

        
