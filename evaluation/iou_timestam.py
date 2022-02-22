 # -*- coding: utf-8 -*
'''
遍历标注文件，按照时间戳对聚类结果进行查找计算IOU
'''

import numpy as np
import os

def getbox():
    '''
	提取存在标注文件的聚类结果数据
	'''
    file_num = 0
    name = np.array([])
    label_file_all = os.listdir(label_file_path)
    label_file_all.sort(key=lambda x:float(x[:-4]))
    for filename in label_file_all:
        star = filename.replace('.txt', '')
        star = float(star) # 取出时间戳

        if dataset == 'eult':
            star = star / 1000000 
        
        name = np.append(name, star)
        file_num += 1
    row_data = np.loadtxt(cluster_file_path, delimiter=' ', usecols=(1, 2, 3, 4, 5, 6, 7))  # 取出数据

    if dataset == 'kitti':
        row_data[:, 0] = np.trunc(row_data[:, 0] * 1000000)
    
    index = np.array([])
    for i in range(file_num):
        ind = np.where(row_data[:, 0] == name[i]) #以标注文件为准把时间戳相等的聚类结果索引提取出来
        index = np.append(index,ind[0]) #把索引ind存入索引数组index
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

    data_labeled = getbox()
    iou_list = np.array([])
    skip_frame = np.array([])
    name_zero = np.array([]) # 存储出现 IOU = 0 的时间戳
    # 按照标注文件进行遍历计算IOU
    label_file_all = os.listdir(label_file_path)
    label_file_all.sort(key=lambda x:float(x[:-4]))
    for filename in label_file_all:
        labels = np.loadtxt(label_file_path + filename, delimiter=' ', usecols=(4, 5, 6, 7, 8, 9))  # 取出标注
        star = filename.replace('.txt','')
        star = float(star)

        if dataset == 'eult':
            star = star / 1000000

        skip_flag = 0
        iou1 = np.array([])  # 存放各组标注对应的IOU
        if np.size(labels) == 6: #如果该时间戳只有1行
            flag1 = 0
            max1 = 0
            for j in data_labeled:
                if j[0] == star:
                    flag1 = 1
                    j_data = np.delete(j, 0, axis=0)  #删除时间戳行
                    iou_res = iou(labels, j_data)
                    iou1 = np.append(iou1, iou_res) #这一行所有IOU值表
            if flag1 == 1:
                skip_flag = 1
                max1 = np.max(iou1)
                if max1 == 0:
                    name_zero = np.append(name_zero, star)
                    iou_list = np.append(iou_list, max1)
                else:
                    iou_list = np.append(iou_list, max1)
            else:                
                name_zero = np.append(name_zero, star)
                iou_list = np.append(iou_list, 0)
        else:
            for row in labels:
                flag2 = 0
                max2 = 0
                iou2 = np.array([])  # 存储每一行标注数据与算法中的众多框的IOU值
                for j in data_labeled:
                    if j[0]==star:
                        flag2 = 1
                        j_data = np.delete(j, 0, axis=0)  # 删除时间戳行
                        iou_res = iou(row, j_data)  # 计算每一行与data_labeled中的IOU值
                        iou2 = np.append(iou2, iou_res)
                if flag2 ==1:
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
    print('丢失的帧数：', len(skip_frame))
   
if __name__ =='__main__':
    dataset = 'eult' # lcas, eult, kitti
    # label_file_path = './iou/lcas_labels/' #标注数据的文件夹地址
    # cluster_file_path = './iou/lcas_results/run_clustering_lcas.txt'#算法输出文件的地址
    label_file_path = './iou/utbm_labels_car/' #标注数据的文件夹地址
    cluster_file_path = './iou/utbm_results/euclidean_clustering_utbm.txt'#算法输出文件的地址
    # label_file_path = './iou/kitti_labels_new/' #标注数据的文件夹地址
    # cluster_file_path = './iou/kitti_results/adaptive_clustering_kitti.txt'#算法输出文件的地址
    IOU_3D = result()
