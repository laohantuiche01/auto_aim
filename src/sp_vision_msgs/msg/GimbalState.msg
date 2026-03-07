std_msgs/Header header
float64[4] quaternion     # wxyz, 云台姿态
float64 yaw               # rad, 偏航角
float64 pitch             # rad, 俯仰角
float64 yaw_vel           # rad/s, 偏航角速度
float64 pitch_vel         # rad/s, 俯仰角速度
float64 bullet_speed      # m/s, 弹速
uint16 bullet_count       # 剩余子弹数
uint8 mode                # 0:IDLE, 1:AUTO_AIM, 2:SMALL_BUFF, 3:BIG_BUFF
