import os
import cv2
import numpy as np

import torch
import random
'''
从文件夹中读数据
'''
class TensorDataset():
    def __init__(self, path, img_width, img_height, aug=False):
        assert os.path.exists(path), "{}路径文件不存在".format(path)

        self.aug = aug
        self.path = path
        self.data_list = []
        self.img_width = img_width
        self.img_height = img_height
        self.img_formats = ['bmp', 'jpg', 'jpeg', 'png']

        # 数据检查
        with open(self.path, 'r') as f:
            for line in f.readlines():
                data_path = line.strip()
                if os.path.exists(data_path):
                    img_type = data_path.split(".")[-1]
                    if img_type not in self.img_formats:
                        raise Exception("图像格式错误:{}".format(img_type))
                    else:
                        self.data_list.append(data_path)
                else:
                    raise Exception("找不到对应的图像 {}".format(data_path))

    def __getitem__(self, index):
        img_path = self.data_list[index]
        label_path = img_path.split(".")[0] + ".txt"

        # 加载图片
        img = cv2.imread(img_path,flags=cv2.IMREAD_GRAYSCALE)
        a = img[np.newaxis,:]
        

        # 加载label文件
        if os.path.exists(label_path):
            label = []
            with open(label_path, 'r') as f:
                for line in f.readlines():
                    l = line.strip().split(" ")
                    label.append([l[0], l[1]])
            label = np.array(label, dtype=np.float32)
        else:
            raise Exception("找不到对应的标签文件{}".format(label_path)) 
        return torch.from_numpy(a), torch.from_numpy(label)

    def __len__(self):
        return len(self.data_list)
