# 用于回传给Identification用来绘图
# 针头匹配
std_msgs/Header header
# ===== 数值记录 =====

# 滤波点世界坐标系
geometry_msgs/Point filtered_point_world
# 目标耗时ms
float32 time_cost
bool solve_ok
# 中心点的数据
geometry_msgs/Point center_world
float32 center_r
geometry_msgs/Vector3 center_velocity
float32 center_l
float32 center_h
bool is_lost
float32 raw_yaw
float32 filter_yaw
# 跟踪状态（与 TrackerState 枚举对应）
uint8 tracking_state

# =====预测可视化=======
# 四个预测装甲板 xyza，中心点为世界坐标系，yaw 为世界系装甲板朝向
geometry_msgs/Point[] predicted_armor_points_world
float32[] predicted_armor_yaws_world
# 当前 EKF 选中的预测装甲板 id，未选中时为 -1
int32 selected_armor_id
# 当前跟踪目标的 ArmorName 枚举值（1-5），NONE=0
int32 armor_name
