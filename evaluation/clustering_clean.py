 # -*- coding: utf-8 -*

import numpy as np
import os

cluster_file_path = './iou/output.txt'

with open(cluster_file_path,'r') as r:
    lines=r.readlines()
with open(cluster_file_path,'w') as w:
    # for l in lines:
    #    if 'intensity' not in l:
    #       w.write(l)
    for l in lines:
        if '[0;m' in l:
            l = l.replace('[0;m', '')
        w.write(l)
print('done')
