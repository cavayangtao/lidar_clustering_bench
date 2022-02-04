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
    for filename in os.listdir(label_file_path):
        star = filename.replace('.txt', '')
        star = float(star)
        # below line is for eu data
        # star = star / 1000000
        name = np.append(name, star)
        file_num += 1
    
    # 将全部点云数据时间戳存入数组
    frame_all =  np.array([])
    for frame in os.listdir(all_frame_path):
        star = frame.replace('.pcd', '')
        star = float(star)
        # below line is for eu data
        # star = star / 1000000
        frame_all = np.append(frame_all, star)

    row_data = np.loadtxt(cluster_file_path, delimiter=' ', usecols=(0, 1, 2, 3, 4, 5, 6))  # 取出聚类数据
    index = np.array([])
    for i in range(file_num):
        ind_name = np.where(frame_all == name[i])
        ind = np.where(row_data[:, 0] == ind_name[0][0]) #以标注文件为准把时间戳相等的聚类结果索引提取出来
        index = np.append(index, ind[0]) #把索引ind存入索引数组index
    outcome = row_data[index.astype(np.int), :]
    return outcome

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
    outcome = getbox_test()

    # 将所有点云文件时间戳存入数组
    frame_all =  np.array([])
    for frame in os.listdir(all_frame_path):
        star = frame.replace('.pcd', '')
        star = float(star)
        # below line is for eu data
        # star = star / 1000000
        frame_all = np.append(frame_all, star)

    iou_list1 = np.array([]) #总的IOU数组（name1行），即每个时间戳的IOU值，共a个
    iou_list2 = np.array([]) #总的IOU数组（name非1行），即每个时间戳的IOU值，共b个【a+b=172】
    name1_zero = np.array([])
    name2_zero = np.array([])
    # 按照标注文件进行遍历计算IOU
    for filename in os.listdir(label_file_path):
        orgin_data = np.loadtxt(label_file_path + filename, delimiter=' ', usecols=(4, 5, 6, 7, 8, 9))  # 取出标注数据
        star = filename.replace('.txt','')
        star = float(star)
        # below line is for eu data
        # star = star / 1000000
        ind_name = np.where(frame_all == star)
        iou_one = np.array([])  # 记录只有1行标注的iou值群
        max_one = 0.0
        max_none = np.array([])
        iou_none = np.array([])  # 存储每一行标注数据与算法中的众多框的IOU值
        iou_none2 = np.array([])  # 存储每个时间戳的IOU值，其元素个数为当前时间戳标注框的个数
        if np.size(orgin_data) == 6: #如果该时间戳只有1行
            for j in outcome:
                flag1 = 0
                if j[0]==ind_name[0][0]:
                    flag1 = 1 #判断循环是否进入的标志
                    j_data = np.delete(j, 0, axis=0)  #删除时间戳行
                    one_iou = iou(orgin_data, j_data)
                    iou_one = np.append(iou_one,one_iou) #这一行所有IOU值表
                    max_one = np.max(iou_one)
            if(flag1 == 1):
                if(max_one==0.0):
                    name1_zero = np.append(name1_zero,star)
                iou_list1 = np.append(iou_list1, max_one)
            else:
                name1_zero = np.append(name1_zero,star)
                iou_list1 = np.append(iou_list1, 0)
        else:
            for hang in orgin_data:
                flag2 = 0
                for j in outcome:
                    if(j[0]==ind_name[0][0]):
                        flag2 = 1
                        j_data = np.delete(j, 0, axis=0)  # 删除时间戳行
                        none_iou = iou(hang, j_data)  # 计算每一行与outcome中的IOU值
                        iou_none = np.append(iou_none, none_iou)
                        max_none = np.max(iou_none) #取该行iou最大值
                if flag2 == 1:
                    if (max_none == 0.0):
                        name2_zero = np.append(name2_zero, star)
                    iou_none2 = np.append(iou_none2, max_none)  # 该时间戳getbox每行的IOU值表
                else:
                    name2_zero = np.append(name2_zero, star)
                    iou_none2 = np.append(iou_none2, 0)
            iou_list2 = np.append(iou_list2, iou_none2)

    print(iou_list1) #只有1行数据的IOU值集合
    print(iou_list2) #多于1行数据的IOU值集合
    print(len(iou_list1))
    print(len(iou_list2))
    print(len(iou_list1)+len(iou_list2)) #程序没错的情况下该项输出值为标注的聚类框个数

    iou_list_all = np.append(iou_list1, iou_list2)
    iou_mean = sum(iou_list_all)/len(iou_list_all)
    print('总的IOU值为：', iou_mean)
    # print('IOU为0的时间戳（1行）：',name1_zero)
    # print('IOU为0的时间戳（非1行）',name2_zero)
    # name_zero = np.append(name1_zero,name2_zero)
    # print(len(name2_zero))
    # print('IOU为0的时间戳:', name_zero)
    # np.savetxt(X=name_zero, fname='/media/yaodexin/TendernessMook/ubuntu_yao/label_cloud/name_zero.txt')

if __name__ =='__main__':
    all_frame_path = '/home/tianbot/Downloads/iou/lcas_points' #所有点云数据
    label_file_path = '/home/tianbot/Downloads/iou/lcas_labels/' #标注数据的文件夹地址
    cluster_file_path = '/home/tianbot/Downloads/iou/lcas_results/depth_clustering_lcas.txt'#算法输出文件的地址
    IOU_3D = result()
