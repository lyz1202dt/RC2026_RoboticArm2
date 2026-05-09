# 河北科技大学2026 RC Robocon 六轴机械臂+KDL解算+Mujoco仿真方案

# 手动输入关节角度，控制机械臂运动。
1. . install/setup.bash && ros2 run arm_calc arm_calc_test
2. 在rqt中 Plugins -> Configuration -> ... -> arm_test_node 设置关节角度，点击 publish_joint_target .
3. arm_calc_node 里面调整 motion_mode , ececute_trajectory 设为真

或者更简单的方法：
启动 arm_task_move_test.launch.py:1。在工作区已经编译过的前提下，先 source 你的 ROS 2 环境和 install 目录，然后执行 ros2 launch launch_pack arm_task_move_test.launch.py。这个 launch 会先拉起仿真，再延时启动 move_kfs_test.cpp:1 里的测试节点。



