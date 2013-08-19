#!/usr/bin/python

import os.path, os

DIR = '/home/dlu/Desktop/path_data'

def remove_no_match(folder, D):
    if not os.path.exists(folder):
        return
    for x in os.listdir(folder):
        base = x[:-5]
        if base in D:
            continue

        full = '%s/%s'%(folder, x)
        print 'rm %s'%full
        os.remove(full)

for folder, folders, files in os.walk(DIR):
    if '.cache' not in folders and '.results' not in folders:
        continue
    D = {}
    for f in files:
        D[f] = {}
        
    remove_no_match(folder + '/.cache', D)
    remove_no_match(folder + '/.results', D)
