from path_planning_analysis import *
import os.path
import yaml
import hashlib

from geometry_msgs.msg import Pose2D
from path_planning_analysis.basic_stats import *
from path_planning_analysis.obstacle_stats import *

def md5_for_file(fn, block_size=2**20):
    md5 = hashlib.md5()
    f = open(fn)
    while True:
        data = f.read(block_size)
        if not data:
            break
        md5.update(data)
    return md5.digest()

def stat_functions():
    fnes = []
    for name in globals().keys():
        value = globals()[name]
        if value.__doc__ and 'METRIC' in value.__doc__:
            fnes.append(value)
    return fnes

class PathStats:
    def __init__(self, filename):
        self.filename = filename
        folder = os.path.dirname( os.path.abspath(filename) )
        self.cachefile = folder + '/.cache/' + filename + '.yaml'
        code = md5_for_file(filename)

        read = False
        if os.path.exists(self.cachefile):
            self.path = None
            data = yaml.load(open(self.cachefile))
            if 'hash' not in data or code != data['hash']:
                read = True
        else:
            read = True
            
        if read:
            self.path = RobotPath(self.filename)
            data = self.path.get_data()
            data['hash'] = code
            yaml.dump( data, open(self.cachefile, 'w'))

        for k,v in data.iteritems():
            if 'poses' in k:
                vals = []
                for p in v:
                    p = Pose2D(p[0], p[1], p[2])
                    vals.append(p)
                v = vals
            elif 'pose' in k:
                v = Pose2D(v[0], v[1], v[2])
            setattr(self, k, v)

    def get_scenario_name(self):
        return self.filename.split('-')[0]

    def get_algorithm(self):
        return self.filename.split('-')[1]

    def stats(self):
        fnes = stat_functions()
        values = {}
        for fn in fnes:
            values[fn.__name__] = fn(self)
        return values

    def get_distance_to_goal(self, index=-1):
        return dist(self.goal_pose, self.poses[index])        

    def get_angle_to_goal(self, index=-1):
        return a_dist(self.goal_pose, self.poses[index])

