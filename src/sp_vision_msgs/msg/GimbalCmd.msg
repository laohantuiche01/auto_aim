std_msgs/Header header
float64 yaw               # rad, 目标偏航角
float64 yaw_vel           # rad/s, 目标偏航角速度
float64 pitch             # rad, 目标俯仰角
float64 pitch_vel         # rad/s, 目标俯仰角速度
uint8 mode                # 0:不控制, 1:仅控制, 2:控制+开火
bool fire                 # 是否开火

