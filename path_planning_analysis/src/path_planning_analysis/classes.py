import collections

from path_planning_analysis.path_stats import *

def get_stats(bags, headers, grouping=None, only_completed=False, tabs=False):
    data = collections.defaultdict(list)
    for filename in bags:
        path = PathStats(filename)  
        
        row = []
        if grouping == 'algorithm':
            key = path.get_unique(tabs)
        elif grouping == 'scenario':
            key = path.get_scenario_name()
        else:
            key = filename

        stats = path.full_stats()
        if only_completed and stats['completed'] < 1.0:
            continue

        for name in headers:
            row.append(stats[name])

        data[key].append(row)
    return data


def rotate_stats(group_data, headers, filter_minimum=0):
    rotated = {}
    for key in sorted(group_data):
        rows = group_data[key]

        data = collections.defaultdict(list)

        for row in rows:
            for header, value in zip(headers, row):
                if value is not None:
                    data[header].append(value)

        data['count'] = len(rows)
        if data['count'] > filter_minimum:
            rotated[key] = data

    return rotated
