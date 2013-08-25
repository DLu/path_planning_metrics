#!/usr/bin/python

import roslib; roslib.load_manifest('path_planning_analysis')
import sys
from path_planning_analysis.interactive import *
from path_planning_analysis.classes import *
from path_planning_analysis.utils import *
from path_planning_analysis.math_util import *
import collections
import rpy
from rpy import r

def t_test(a,b,one_sided=True):
    try:
        if one_sided:
            X = r.t_test(a,b, alternative="greater")
        else:
            X = r.t_test(a,b)
    except:
        return {'p': '!'}

    return {'p':  X['p.value'], 
            't':  X['statistic']['t'],
            'm1': X['estimate']['mean of x'],
            'm2': X['estimate']['mean of y']}


if __name__=='__main__':
    headers, bags = analysis_argparse()
    
    group_data = get_stats(bags, headers, 'algorithm')

    data = rotate_stats(group_data, headers)
    
    pdata = []

    algorithms = data.keys()

    for header in headers:
        print header + "=" * 50
        seen = set()
        sdata = []
        sdata.append( [''] + algorithms )
        for a1 in algorithms:
            row = [a1]
            for a2 in algorithms:
                if a1>=a2:
                    row.append('')
                    continue

                vals1 = data[a1][header]
                vals2 = data[a2][header]
                if len(vals1)==0 or len(vals2)==0:
                    row.append('x')
                    continue

                T = t_test(vals1, vals2,False)

                row.append(T['p'])
            sdata.append(row)
        print_table(sdata, False, 4)
