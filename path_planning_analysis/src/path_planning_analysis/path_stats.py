from path_planning_analysis import *
import os.path
import yaml
import hashlib

from geometry_msgs.msg import Pose2D
from path_planning_analysis.basic_stats import *
from path_planning_analysis.obstacle_stats import *
from path_planning_analysis.social_stats import *
from path_planning_analysis.time_stats import *
from path_planning_analysis.path_analyze import *
from path_planning_analysis.array_stats import *
import os, errno
import sys

def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

def md5_for_file(fn, block_size=2**20):
    md5 = hashlib.md5()
    f = open(fn)
    while True:
        data = f.read(block_size)
        if not data:
            break
        md5.update(data)
    return md5.digest()

def gather_functions(key):
    fnes = []
    for name in globals().keys():
        value = globals()[name]
        if value.__doc__ and key in value.__doc__:
            fnes.append(value)
    return fnes

def stat_functions():
    return gather_functions('METRIC')
    
def array_functions():
    return gather_functions('ARRAY_MET')


class PathStats:
    def __init__(self, filename):
        self.filename = filename
        self.features = path_analyze(filename)
        folder, basefile = os.path.split( os.path.abspath(filename) )
        mkdir_p( folder + '/.cache/' )
        mkdir_p( folder + '/.results/' )
        self.cachefile = folder + '/.cache/' + basefile + '.yaml'
        self.resultsfile = folder + '/.results/' + basefile + '.yaml'

        self.path_ready = False
        self.data_fields = []
        
        code = md5_for_file(self.filename)
        if os.path.exists(self.resultsfile):
            self.results = yaml.load(open(self.resultsfile))
            if not self.results:
                self.results = {}
                
            if self.results.get('hash', '') != code:
                self.results = {'hash': code}
        else:
            self.results = {'hash': code}
            
    def load(self, force_cache=False):
        if self.path_ready and not force_cache:
            return
        code = md5_for_file(self.filename)

        read = False
        if os.path.exists(self.cachefile):
            self.path = None
            data = yaml.load(open(self.cachefile))
            if 'hash' not in data or code != data['hash']:
                read = True
        else:
            read = True
            
        if read:
            sys.stderr.write("Reading bag file for %s\n"%self.filename)
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
            if type(v)==list and len(v)>0 and type(v[0])==float:
                self.data_fields.append(k)
                
        for fne in array_functions():
            k = fne.__name__
            setattr(self, k, fne(self))
            self.data_fields.append(k)
            
        self.path_ready = True

    def get_scenario_name(self):
        return self.features['scenario']

    def get_algorithm(self):
        return self.features['algorithm']

    def stats(self):
        fnes = stat_functions()
        values = {}
        for fn in fnes:
            name = fn.__name__
            
            if name in self.results:
                result = self.results[name]
            else:
                self.load()
                result = fn(self)
                self.results[name] = result

            if type(result)==type({}):
                values.update(result)
            else:
                values[name] = result

        yaml.dump( self.results, open(self.resultsfile, 'w'))
        return values

    def full_stats(self):
        m = {}
        m.update(self.stats())
        m.update(self.features)
        return m

    def get_distance_to_goal(self, index=-1):
        return dist(self.goal_pose, self.poses[index])        

    def get_angle_to_goal(self, index=-1):
        return a_dist(self.goal_pose, self.poses[index])

    def get_num_poses(self):
        return len(self.poses)

