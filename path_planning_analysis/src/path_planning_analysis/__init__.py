import roslib; roslib.load_manifest('path_planning_analysis')
import rosbag
import collections
import pylab
from math import sin, cos, sqrt

def derivative(t, x, window=2):
    N = window * 2 + 1
    tarr = []
    arr = []
    for ti, xi in zip(t,x):
        tarr.append(ti)
        arr.append(xi)
        while len(tarr) > N:
            tarr = tarr[1:]
            arr = arr[1:]

def dist(p1, p2):
    return sqrt( pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) )
        

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
        pylab.plot(ts, ds)
        pylab.show()
        return ts, ds

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
        ts=[]
        for t, pose in self.poses:
            ts.append(t.to_sec())
            x.append(pose.x)
            y.append(pose.y)
        ax.plot(ts,x)
        ax.plot(ts,y)
        pylab.show()
        
