from path_planning_analysis import *
import os.path
import yaml
import hashlib

def md5_for_file(fn, block_size=2**20):
    md5 = hashlib.md5()
    f = open(fn)
    while True:
        data = f.read(block_size)
        if not data:
            break
        md5.update(data)
    return md5.digest()

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
            setattr(self, k, v)

    def stats(self):
        return []
