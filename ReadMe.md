# 总流程(基础)
相机获取图像 -> 对获取图片的预处理 -> 进行灯条匹配算法 -> 实现追踪功能 -> 解算 pitch 和 yaw -> 通过串口传递给电控。
目前阶段在 获取相机获取图像 和 预处理阶段。

主要由两个文件
1. ArmorPlateIdentifcation.cpp -> 以相机为中心的核心节点
2. Test.cpp -> 以视频来判断是否可行的

还有一个以往参考文件
1. DetectorDriver.cpp
# 获取图像
## 核心关键
相机的参数配置
我这里属于薄弱点，基本对相机的使用几乎没有。几乎全靠ai辅助写的
### 目前的大概区间是
曝光 -> 100 - 300 us
增益 -> 20

其他参数还没有研究
# 对图片的预处理
## 核心参数
1. 一个flag来判断识别蓝色还是红色 
2. 返回值是一个二值化的图像/边框用于进一步的识别
## 目前进度
## 核心方法
### 颜色检查
主要实现
采用 R-B  / B - R
-  R - B 用于检测 Red灯条
- B - R 用于检测 Blue灯条

我测试过 HSV 这个方法不要用
# 二值化
通过 R-B B-R通道后得到的图像之后，
开始用 大津法 来直接二值化
### 处理
网上说使用膨胀。
## 目前进度
还在尝试使用先检测其中一种颜色。然后寻找更多的方法测试。
目前一种颜色成功
# 进行灯条匹配算法
## 相关类
1. Lights
2. PairedLights
### Lights
包含灯条的信息
中心点， 得到的旋转矩形， 上下两个点， 是否匹配的信息
### PairedLights
主要的存储匹配好的灯条和算法的类
包含 灯条数量，匹配好的灯条
## 子流程
找到灯条的轮廓 -> 将轮廓匹配为直线(两个角点) -> 灯条点位匹配机制
### 灯条轮廓
主要实现 
findContours

实现最初的筛选工作
1. 最基本的面积筛选（防止面积太小了）
2. 最基本都长宽比的筛选（防止出现超长噪音）
## 将轮廓匹配为直线(两个角点)
主要实现
minAreaRect 和fitEllipse
通过 fitEllipse 提供**角度**信息，
再通过minAreaRec提供**长度**信息

## 灯条点位匹配机制
### 主要思路
1. 先通过6个经验参数来实现初步约束，找到装甲板
2. 正对于多个匹配的装机版采取 匹配 - 打分 - NMS 进一步匹配
### 经验参数
- 角度差(平行程度)
- 灯条长度相似度
- 中心距离之比
- y差比
- x差比
其中 y , x差比则是在以左边的灯的参考系下计算
这样避免相机坐标系导致的 倾斜装甲板 导致y差比率和x差比率的不合理
### 打分参数
总分3分，每个1分
- 角度差(平行程度)
- 灯条长度相似度
- 装机版长宽比形状分
## 缺陷(往往没想好解决办法)
1. 在60度左右下，因为保证取向的问题，所以会top和bottom不准。（无能为力）
 否则就是删掉
``` cpp
   else if(dir.x > 0) {
	dir = -dir;
}
```
使得问题限制在90°
2. 装甲板出现俯仰的时候，会导致角度差过大导致识别失败
## 下一步改进
1. 对打分机制的权重,比如说 灯条长度相似度 这条提高
2. 提高筛选条件 -- 目前条件比较松
# 位姿解算
## 核心计算
opencv中SlovePnP
通过PnP解算出来的数据算出来 yaw 和 pitch
# 实现追踪功能
先针对线性卡尔曼滤波进行更新。正对yaw和pitch单独更新。
其中中断秒数为0.5s。
目前只在视频实现，还未在相机中使用，请参考test.cpp
目前版本针对的是对多个装甲板追踪的功能中选择历史连续装甲板。
如果有突变的话就是选择新的。
## 
# 使用教程
## 正常模式
1. 功能包构建
2. 启动launch文件
	- 启动相机控制`ros2 launch armor_plate_identification run.launch.py `
	- 启动视频控制`ros2 launch armor_plate_identification test.launch.py`
## DEBUG模式
### 开启全部debug模式
1. 功能包的构建
``` bash
colcon build --packages-select armor_plate_identification --cmake-args -DDEBUG_MODE=ON
```
这里如果是对应的debug模式的话，请对应的debug
比如说
- DEBUG_INDENTIFICATION
- DEBUG_PREPROCESSING
## 时钟不匹配
当出现这种情况下，请输入
```bash
rm -rf build/armor_plate_identification install/armor_plate_identification
colcon build --packages-select armor_plate_identification --cmake-args -DDEBUG_MODE=ON
```
