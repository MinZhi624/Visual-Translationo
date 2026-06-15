# GUI 调试窗口颜色注册表

防止后续修改时颜色冲突。所有颜色为 BGR 格式。

---

## identification 窗口

品红 (255, 0, 255) — 装甲板 X 形对角线，始终显示
浅黄绿 (207, 216, 129) — 灯条旋转矩形轮廓，debug_lights_ 开启时
黄色 (0, 255, 255) — 数字识别结果 "3 (95%)"，debug_number_classification_ 开启时
橙色 (0, 165, 255) — 处理耗时 "Process: XX ms"，左上角 (10, 30)，始终显示
黄色 (0, 255, 255) — 播放延迟 "Delay: XX ms"，左上角 (10, 55)，debug_timecontrol_ 开启时

---

## tracker_debug 窗口

图像缩放 0.5x 显示。叠加 Tracker 和 Planner 两层信息。

### Tracker 层

白色 (255, 255, 255) — 追踪状态文字 "LOST/DETECTING/TRACKING/TEMP_LOST"，左上角 (10, 25)
红色 (0, 0, 255) — EKF 车体旋转中心，实心圆半径 5px
黄色 (0, 255, 255) — 4 块预测装甲板矩形 + 编号 0~3，线宽 2

### Planner 层

红色 (0, 0, 255) — "PLANNER INVALID" 文字，位置 (10, 55)，弹道无效时
绿色 (0, 255, 0) — 原始目标点 (original)，实心圆半径 5px
黄色 (0, 255, 255) — 预测点 (predicted)，实心圆半径 5px
蓝色 (255, 0, 0) — 弹道补偿点 (compensated)，实心圆半径 5px
绿色 (0, 255, 0) — 原始→预测连线，线宽 2
黄色 (0, 255, 255) — 预测→补偿连线，线宽 2

---

## preprocess 窗口

2x2 网格，每格 640x480，标题栏 30px。debug_preprocessing_ 开启时显示。

黄色 (0, 255, 255) — 四个格子的标题文字
绿色 (0, 255, 0) — 碎片标注，单个碎片（正常）
红色 (0, 0, 255) — 碎片标注，多个碎片（异常）

布局：
  左上 Original | 右上 BLUE_dim (fragments)
  左下 GRAY_thre | 右下 Merged

---

## rejected_rois 窗口

无额外颜色标注，仅水平拼接被拒绝的数字 ROI。debug_number_classification_ 开启时显示。

---

## number_rois 窗口

无额外颜色标注，仅水平拼接待识别的数字 ROI。debug_number_classification_ 开启时显示。

---

## RViz Marker（armor_plate_tracker 发布）

绿色 (0, 1, 0) — 车体旋转中心球体 + 旋转轴箭头 + 4 块预测装甲板
黄色 (1, 1, 0) — 车体速度方向箭头
白色 (1, 1, 1) — 装甲板编号文字 "id=0" ~ "id=3"
红色 (1, 0, 0) — 观测/测量装甲板
蓝色 (0, 0, 1) — 滤波后装甲板
