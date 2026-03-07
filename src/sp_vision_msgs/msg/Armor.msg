std_msgs/Header header

# YOLO 检测输出（必需）
int32 class_id            # 类别ID，计算公式：color * 18 + name * 2 + type
                          # color: red=0, blue=1, extinguish=2, purple=3
                          # name: one=0, two=1, three=2, four=3, five=4, sentry=5, outpost=6, base=7, not_armor=8
                          # type: big=0, small=1

# PnP 求解输入（必需）
geometry_msgs/Point32[4] keypoints  # 4个角点像素坐标，顺序：左上、右上、右下、左下 (tl, tr, br, bl)

# 调试/可视化字段（可选，与 class_id 冗余）
string number             # 装甲板数字名称 ("1"-"5", "sentry", "outpost", "base", "unknown")
string type               # 装甲板类型 ("big", "small")
uint8 color               # 颜色枚举 (0:RED, 1:BLUE, 2:EXTINGUISH, 3:PURPLE)

# PnP 求解输出（后端填充）
geometry_msgs/Pose pose   # 3D位姿：position为相机坐标系下的位置(m)，orientation为四元数
