1、快速行程终点，垂直方向在KFS上沿上方50mm……水平方向终点是面对的KFS那一面的位置偏向机械臂底座10mm。
2、慢速行程轨迹是垂直向下的。
3、回缩动作耗时压缩到1s以内。



Launch文件	用途	启动的节点
arm_mujoco_sim.launch.py	MuJoCo仿真环境	robot_state_publisher, mujoco_ros2_control, dog_controller, arm_calc, rviz2
arm_real.launch.py	真实机器人完整运行	arm_driver, robot_state_publisher, arm_calc, rviz2, vision, arm_task, camera TF
arm_real_target_test.launch.py	真实机器人目标测试	与arm_real类似，但增加了静态target_object TF（用于测试抓取目标位置）
arm_static_display.launch.py	URDF模型静态显示	robot_state_publisher, joint_state_publisher_gui, rviz2（用于可视化调试模型）
arm_task_sim.launch.py	任务仿真	包含arm_mujoco_sim + static_tf_camera + arm_task
arm_task_target_test.launch.py	任务目标测试	包含arm_task_sim + 静态target_object TF + catch_kfs_test节点
vision_tf_test.launch.py	视觉TF测试	arm_driver, robot_state_publisher, arm_calc, rviz2, vision, camera TF（不含arm_task）