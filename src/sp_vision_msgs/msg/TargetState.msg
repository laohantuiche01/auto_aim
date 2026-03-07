std_msgs/Header header
uint8 number              # 装甲板编号
uint8 type                # 装甲板类型
float64[3] position       # 位置 (x, y, z) in world
float64[3] velocity       # 速度 (vx, vy, vz) in world
float64 yaw               # 偏航角
float64 yaw_vel           # 偏航角速度
float64[3] ypr_in_camera  # 在相机坐标系下的姿态
uint8 color               # 颜色
