 # -*- coding: utf-8 -*

import numpy as np

time_file_path = './iou/output.txt'
output_path = './iou/output1.txt'
time_frame = 0
w =  open(output_path,'w')
with open(time_file_path,'r') as r:
    lines=r.readlines()
    for l in lines:
        if 'Ground removed in' in l:
            l = l.split("in ")[1]
            l = l.split(" us")[0]
            l = int(l)
            time_frame = time_frame + l
        elif 'image based labeling took:' in l:
            l = l.split("took: ")[1]
            l = l.split(" us")[0]
            l = int(l)
            time_frame = time_frame + l
        elif 'labels image sent to clients in:' in l:
            l = l.split("in: ")[1]
            l = l.split(" us")[0]
            l = int(l)
            time_frame = time_frame + l
        elif 'prepared clusters in:' in l:
            l = l.split("in: ")[1]
            l = l.split(" us")[0]
            l = int(l)
            time_frame = time_frame + l
        elif 'clusters shared:' in l:
            l = l.split("shared: ")[1]
            l = l.split(" us")[0]
            l = int(l)
            time_frame = time_frame + l
        elif '******' in l:
            time_frame = time_frame/1000000.0
            print(time_frame)
            w.write(str(time_frame))
            w.write('\n')
            time_frame = 0
w.close
