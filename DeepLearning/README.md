# 装甲板数字识别器

有适配rm_vision项目的example，但由于我们战队没用ROS2，暂时无法测试


### 0. 安装依赖

```
pip install -r requirements.txt
```

### 1. 准备数据集 

我们自己制作的数据集在dataset/armors.zip里，约有1.6w张图片，大小均为28x28，每张图片有两个标签，第一个标签为0或1，对应负样本和真实装甲板，第二个标签为0-9,对应标签为 0:负样本 1-5:1-5号装甲板 6:老哨兵 7:前哨站 8:基地 9:新哨兵

使用make_txt.py来生成一个txt文件方便Dataloader读取，记得修改make_txt.py中的path为你的路径


### 2. 训练

修改train.py中main()的cfg_path为armors.txt的路径，运行

```
python train.py
```

