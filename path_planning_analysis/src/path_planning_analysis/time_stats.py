from path_planning_analysis.math_util import *

def time_stats(times):
    return min_max_avg(times)

def all_time_stats(path):
    """METRICS"""
    names = ['global', 'local']
    data = [path.global_times, path.local_times]
    stats = {}

    for name, d in zip(names, data):
        for stat,sname in zip(time_stats(d), ('min', 'max', 'average')):
            stats[ '%s_%s_update_time'%(sname, name) ] = stat
    return stats

