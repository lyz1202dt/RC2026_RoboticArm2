builtin_interfaces/Time stamp
int32 arm_id    # 目标机械臂ID（0左1右）
int32 mode      # 0-停机;1-关节空间;2-笛卡尔空间;
std_msgs/Float64MultiArray position     # 按照x,y,z,pitch的顺序填写空间坐标和俯仰角或按照关节顺序从小到大填写关节期望弧度
float32 duration    # 轨迹执行时间
