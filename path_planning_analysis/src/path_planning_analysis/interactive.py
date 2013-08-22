import sys
from path_planning_analysis.path_stats import *

GROUPS = {
'basic': ['time', 'collisions', 'completed'],
'global-update-time': ['min_global_update_time', 'average_global_update_time', 'max_global_update_time'],
'local-update-time': ['min_local_update_time', 'average_local_update_time', 'max_local_update_time'],
'social': ['minimum_distance_to_person', 'average_distance_to_person']
}

def all_keys(filename, arrays=False):
    path = PathStats(filename)  
    stats = path.stats()
    keys = stats.keys()
    if arrays:
        path.load(True)
        print path.data_fields
        keys += path.data_fields
    return keys

def select(options, one=False):
    selected = set()
    values = []

    while True:
        ret = []
        for i, option in enumerate(options):
            if type(option)==type(""):
                key = option
                if option in values:
                    ret.append(option)
                    c = '*'
                else:
                    c = ' '
            else:
                key, arr = option
                if key in values:
                    ret += arr
                    c = '*'
                else:
                    c = ' '
            print "%2d) (%s) %s"%(i, c, key)
        if not one:
            print ' x) Done'
        inp = raw_input()
        if inp=='x':
            break
        else:
            try:
                i = int(inp)
                key = options[i]
                if type(key)!=type(''):
                    key = key[0]

                if one:
                    return [options[i]]

                if key in values:
                    values.remove(key)
                else:
                    values.append(key)
            except ValueError:
                None

    return ret

def analysis_argparse(argv=None, one=False, arrays=False):
    if argv is None:
        argv = sys.argv[1:]

    headers = []
    bags = []

    all_headers = False

    for arg in argv:
        if '.bag' in arg:
            bags.append(arg)
        elif arg[0]=='-':
            if arg[0:2]=='--':
                key = arg[2:]
                if key in GROUPS:
                    headers += GROUPS[key]
                elif key=='all':
                    all_headers = True
        else:
            headers.append(arg)

    if all_headers:
        headers = all_keys(bags[0])

    if len(headers)==0:
        keys = all_keys(bags[0], arrays)
        if one:
            headers = select(sorted(keys), one)
        else:
            headers = select(GROUPS.items() + sorted(keys), one)

    return headers, bags

