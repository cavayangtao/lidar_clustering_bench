# -*- coding: UTF-8 -*-
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

dataset = 'kitti'
num_frame = 161

#这里导入你自己的数据
adaptive_path = './iou/' + dataset + '_results/time_adaptive_clustering.txt'
adaptive = np.loadtxt(adaptive_path)
adaptive = adaptive[:num_frame] * 1000
ave_adaptive = np.around(np.average(adaptive), 2)
std_adaptive = np.around(np.std(adaptive), 2)
med_adaptive = np.around(np.median(adaptive), 2)

autoware_path = './iou/' + dataset + '_results/time_autoware_clustering.txt'
autoware = np.loadtxt(autoware_path)
autoware = autoware[:num_frame] * 1000
ave_autoware = np.around(np.average(autoware), 2)
std_autoware = np.around(np.std(autoware), 2)
med_autoware = np.around(np.median(autoware), 2)

euclidean_path = './iou/' + dataset + '_results/time_euclidean_clustering.txt'
euclidean = np.loadtxt(euclidean_path)
euclidean = euclidean[:num_frame] * 1000
ave_euclidean = np.around(np.average(euclidean), 2)
std_euclidean = np.around(np.std(euclidean), 2)
med_euclidean = np.around(np.median(euclidean), 2)

run_path = './iou/' + dataset + '_results/time_run_clustering.txt'
run = np.loadtxt(run_path)
run = run[:num_frame] * 1000
ave_run = np.around(np.average(run), 2)
std_run = np.around(np.std(run), 2)
med_run = np.around(np.median(run), 2)

depth_path = './iou/' + dataset + '_results/time_depth_clustering.txt'
depth = np.loadtxt(depth_path)
depth = depth[:num_frame] * 1000
ave_depth = np.around(np.average(depth), 2)
std_depth = np.around(np.std(depth), 2)
med_depth = np.around(np.median(depth), 2)

#开始画图
# plt.title('Result Analysis')
plt.plot(run, color='red', label='Run clustering: ' + str(ave_run) + ' ms' + '  +/-  ' + str(std_run) + ' ms')
plt.plot(depth, color='orange', label='Depth clustering: ' + str(ave_depth) + ' ms' + '  +/-  ' + str(std_depth) + ' ms')
plt.plot(euclidean, color='blue', label='Euclidean clustering: ' + str(ave_euclidean) + ' ms' + '  +/-  ' + str(std_euclidean) + ' ms')
plt.plot(adaptive, color='green', label='Adaptive clustering: ' + str(ave_adaptive) + ' ms' + '  +/-  ' + str(std_adaptive) + ' ms')
plt.plot(autoware,  color='skyblue', label='Autoware clustering: ' + str(ave_autoware) + ' ms' + '  +/-  ' + str(std_autoware) + ' ms')
plt.legend() # 显示图例
plt.xlabel('frame number')
plt.ylabel('runtime [ms]')
plt.show()

print(med_run, med_depth, med_euclidean, med_adaptive, med_autoware)