import collections

from path_planning_analysis.path_stats import *

def collect_stats(bags, headers, merge_trials=False, min_trials=0, remove_singular_headers=True):
    groups, metadata = get_raw_stats(bags, headers, merge_trials)
    summed = rotate_stats(groups, headers, min_trials)
    
    constants = {}
    keys = []
    constants, keys = get_parameters(metadata.values())
            
    if merge_trials:
        newh = keys + ['count'] + headers
    else:
        newh = keys + headers    
    
    for k, v in summed.iteritems():
        v.update(metadata[k])
        s = '/'.join([str(v[kk]) for kk in keys])
        v['unique'] = s
        
    
    return newh,summed,constants
    
def get_parameters(array_of_feature_sets, filter_trials=True):
    constants = {}
    keys = []

    V = collections.defaultdict(set)
    for M in array_of_feature_sets:
        for k,v in M.iteritems():
            if filter_trials and k=='trial':
                continue
            V[k].add(v)
            
    for k,v in V.iteritems():
        if len(v)>1:
            keys.append(k)
        else:
            constants[k] = list(v)[0]
    return constants, keys
    

def get_raw_stats(bags, headers, merge_trials=False):
    data = collections.defaultdict(list)
    metadata = {}
    
    for filename in bags:
        path = PathStats(filename)  
        
        row = []
        if merge_trials:
            features = path.features.copy()
            del features['trial']
            key = str(features)
            metadata[key] = features            
        else:
            key = filename
            metadata[key] = {'filename': filename}

        stats = path.full_stats()
        for name in headers:
            row.append(stats[name])

        data[key].append(row)
    return data, metadata

def rotate_stats(groups, headers, filter_minimum=0):
    rotated = {}
    for key in sorted(groups):
        rows = groups[key]
        data = collections.defaultdict(list)

        for row in rows:
            for header, value in zip(headers, row):
                if value is not None:
                    data[header].append(value)

        data['count'] = len(rows)
        if data['count'] > filter_minimum:
            rotated[key] = data

    return rotated
    
def map_string(M):
    s = ''
    for c,v in M.iteritems():
        s += '%s:%s '%(c,v)
    return s.strip()
