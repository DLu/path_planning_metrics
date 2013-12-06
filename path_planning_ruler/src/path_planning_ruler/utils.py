import os, errno
import rosbag

def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

def bag(filename, data):
    b = rosbag.Bag(filename, 'w', compression=rosbag.Compression.BZ2)
    for time, topic, msg in sorted(data):
        b.write(topic, msg, t=time)
    b.close()
