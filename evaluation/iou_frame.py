 # -*- coding: utf-8 -*
'''
遍历标注文件，按照帧号对聚类结果进行查找计算IOU
'''

import numpy as np
import os

def getbox_test():
    '''
	提取存在标注文件的聚类结果数据
	'''
    # 将标注文件名存入数组
    file_num = 0
    name = np.array([])
    label_file_all = os.listdir(label_file_path)
    label_file_all.sort(key=lambda x:float(x[:-4]))
    for filename in label_file_all:
        star = filename.replace('.txt', '')
        star = float(star)
        name = np.append(name, star)
        file_num += 1
    
    # 将全部点云数据时间戳存入数组
    frame_all =  np.array([])
    frame_file_all = os.listdir(all_frame_path)
    frame_file_all.sort(key=lambda x:float(x[:-4]))
    for frame in frame_file_all:
        star = frame.replace('.pcd', '')
        star = float(star)
        frame_all = np.append(frame_all, star)

    row_data = np.loadtxt(cluster_file_path, delimiter=' ', usecols=(0, 1, 2, 3, 4, 5, 6))  # 取出聚类数据
    index = np.array([])
    for i in range(file_num):
        ind_name = np.where(frame_all == name[i])
        ind = np.where(row_data[:, 0] == ind_name[0][0]) #以标注文件为准把时间戳相等的聚类结果索引提取出来
        index = np.append(index, ind[0]) #把索引ind存入索引数组index
    data_labeled = row_data[index.astype(np.int), :]
    return data_labeled

def  iou(box1,box2):
    '''
	3D IoU计算
	box表示形式：[x1,y1,z1,x2,y2,z2] 分别是两对角点的坐标
	'''
    in_w = min(box1[3],box2[3]) - max(box1[0],box2[0])
    in_l = min(box1[4],box2[4]) - max(box1[1],box2[1])
    in_h = min(box1[5],box2[5]) - max(box1[2],box2[2])

    inter = 0 if in_w < 0 or in_l < 0 or in_h < 0 else in_w * in_l * in_h
    union = (box1[3] - box1[0]) * (box1[4] - box1[1]) * (box1[5] - box1[2]) + (box2[3] - box2[0]) * (box2[4] - box2[1]) * (box2[5] - box2[2])  - inter
    iou = inter / union
    return iou

def result():
    data_labeled = getbox_test()
    # 将所有点云文件时间戳存入数组
    frame_all =  np.array([])
    frame_file_all = os.listdir(all_frame_path)
    frame_file_all.sort(key=lambda x:float(x[:-4]))
    for frame in frame_file_all:
        star = frame.replace('.pcd', '')
        star = float(star)
        frame_all = np.append(frame_all, star)

    iou_list = np.array([]) # 存储文件标注的结果
    skip_frame = np.array([])
    name_zero = np.array([])
    # 按照标注文件进行遍历计算IOU
    label_file_all = os.listdir(label_file_path)
    label_file_all.sort(key=lambda x:float(x[:-4]))
    for filename in label_file_all:
        labels = np.loadtxt(label_file_path + filename, delimiter=' ', usecols=(4, 5, 6, 7, 8, 9))  # 取出标注数据
        star = filename.replace('.txt','')
        star = float(star)
        ind_name = np.where(frame_all == star)

        skip_flag = 0
        iou1 = np.array([])  # 存放各组标注对应的IOU
        if np.size(labels) == 6: #如果该时间戳只有1行
            flag1 = 0
            max1 = 0
            for j in data_labeled:
                if j[0] == ind_name[0][0]:
                    flag1 = 1
                    j_data = np.delete(j, 0, axis=0)  #删除时间戳行
                    iou_res = iou(labels, j_data)
                    iou1 = np.append(iou1, iou_res) #这一行所有IOU值表
            if flag1 == 1:
                skip_flag = 1
                max1 = np.max(iou1)
                if max1 ==0:
                    name_zero = np.append(name_zero,star)
                    iou_list = np.append(iou_list, max1)
                else:
                    iou_list = np.append(iou_list, max1)
            else:
                name_zero = np.append(name_zero,star)
                iou_list = np.append(iou_list, 0)
        else:
            for row in labels:
                flag2 = 0
                max2 = 0
                iou2 = np.array([])  # 存储每一行标注数据与算法中的众多框的IOU值
                for j in data_labeled:
                    if j[0]==ind_name[0][0]:
                        flag2 = 1
                        j_data = np.delete(j, 0, axis=0)  # 删除时间戳行
                        iou_res = iou(row, j_data)  # 计算每一行与data_labeled中的IOU值
                        iou2 = np.append(iou2, iou_res)
                if flag2 == 1:
                    skip_flag = 1
                    max2 = np.max(iou2) #取该行IOU最大值
                    if max2 == 0:
                        name_zero = np.append(name_zero, star)
                        iou1 = np.append(iou1, max2)  # 该时间戳getbox每行的IOU值表
                    else:
                        iou1 = np.append(iou1, max2)  # 该时间戳getbox每行的IOU值表
                else:
                    name_zero = np.append(name_zero, star)
                    iou1 = np.append(iou1, 0)
            iou_list = np.append(iou_list, iou1)
        if skip_flag == 0:
            skip_frame = np.append(skip_frame, star)

    iou_mean = sum(iou_list)/len(iou_list)

    print(iou_list)
    print('平均IOU为：', iou_mean)
    print('保存的IOU数：', len(iou_list))
    print('IOU为0的标签数：', len(name_zero))
    print('缺失的帧数：', len(skip_frame))

if __name__ =='__main__':
    all_frame_path = './iou/lcas_points' #所有点云数据
    label_file_path = './iou/lcas_labels/' #标注数据的文件夹地址
    cluster_file_path = './iou/lcas_results/depth_clustering_lcas.txt'#算法输出文件的地址
    # all_frame_path = './iou/utbm_points' #所有点云数据
    # label_file_path = './iou/utbm_labels_car/' #标注数据的文件夹地址
    # cluster_file_path = './iou/utbm_results/depth_clustering_utbm.txt'#算法输出文件的地址
    # all_frame_path = './iou/kitti_points_noground' #所有点云数据
    # label_file_path = './iou/kitti_labels_new/' #标注数据的文件夹地址
    # cluster_file_path = './iou/kitti_results/depth_clustering_kitti.txt'#算法输出文件的地址
    IOU_3D = result()
