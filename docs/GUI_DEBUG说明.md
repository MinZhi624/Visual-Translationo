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

所有 marker 使用 `base_id=0`（car markers），测量和滤波 marker 固定 ID。

ID 0 — 中心点球体，绿色 (0, 1, 0)，半径 0.08m
ID 1 — 旋转轴箭头，绿色 (0, 1, 0)，从中心向上 0.5m
ID 2 — 车体速度箭头，黄色 (1, 1, 0)，从中心沿速度方向，缩放 0.5
ID 3 — 观测装甲板 Box，红色 (1, 0, 0)
ID 4 — 滤波装甲板 Box，蓝色 (0, 0, 1)
ID 5 — 预测装甲板 0 Box，绿色 (0, 1, 0)
ID 6 — 预测装甲板 1 Box，绿色 (0, 1, 0)
ID 7 — 预测装甲板 2 Box，绿色 (0, 1, 0)
ID 8 — 预测装甲板 3 Box，绿色 (0, 1, 0)
ID 9 — 文字标签 "id=0"，白色 (1, 1, 1)
ID 10 — 文字标签 "id=1"，白色 (1, 1, 1)
ID 11 — 文字标签 "id=2"，白色 (1, 1, 1)
ID 12 — 文字标签 "id=3"，白色 (1, 1, 1)

ns: tracker_sphere（ID 0）
ns: tracker_velocity（ID 1, 2）
ns: tracker_box（ID 3~8）
ns: tracker_text（ID 9~12）
