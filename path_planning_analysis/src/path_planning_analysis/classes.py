import collections

from path_planning_analysis.path_stats import *

def get_stats(bags, headers, grouping=None):
    data = collections.defaultdict(list)
    for filename in bags:
        path = PathStats(filename)  
        row = []
        if grouping == 'algorithm':
            key = path.get_algorithm()
        else:
            key = filename

        stats = path.stats()

        for name in headers:
            row.append(stats[name])

        data[key].append(row)
    return data


def rotate_stats(group_data, headers):
    rotated = {}
    for key in sorted(group_data):
        rows = group_data[key]

        data = collections.defaultdict(list)

        for row in rows:
            for header, value in zip(headers, row):
                if value:
                    data[header].append(value)

        data['count'] = len(rows)
        rotated[key] = data

    return rotated
