# 用于回传给Identification用来绘图
# 针头匹配
std_msgs/Header header
# ===== 数值记录 =====

# 目标点世界坐标系
geometry_msgs/Vector3 target_point_world
geometry_msgs/Vector3 filtered_point_world
# 目标耗时ms
float32 time_cost
bool solve_ok
# 中心点的数据
float32 center_x
float32 center_y
float32 center_z
float32 center_r
float32 center_v_x
float32 center_v_y
float32 center_v_z
float32 center_l
float32 center_h
bool is_lost
float32 raw_yaw
float32 filter_yaw

# =====预测可视化=======
# 四个预测装甲板 xyza，中心点为世界坐标系，yaw 为世界系装甲板朝向
geometry_msgs/Vector3[] predicted_armor_points_world
float32[] predicted_armor_yaws_world
# 当前 EKF 选中的预测装甲板 id，未选中时为 -1
int32 selected_armor_id

