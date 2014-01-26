#!/usr/bin/python

import os, os.path
from path_planning_analysis.path_stats import *

for folder, folders, files in os.walk('/home/dlu/Desktop/path_data/'):

    if '.cache' in folder or '.results' in folder:
        continue
    if len(folder)<=29:
        continue
    if folder[28] not in '01':
        continue
        
    for fn in files:
        if '.bag' == fn[-4:]:
            try:
                path = PathStats(os.path.abspath(folder + '/' + fn))  
                path.load(True)
            except:
                None
            

