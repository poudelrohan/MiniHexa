import Hiwonder  # 导入Hiwonder机器人控制库
import time      # 导入时间模块，用于延时控制

# 初始化机器人
robot = Hiwonder.Robot()  # 创建机器人对象

# 设置身体在X轴正方向移动3厘米
robot.set_body_pose([3.0, 0.0, 0.0], 600)
time.sleep(1)  # 等待1秒

# 设置身体在Y轴正方向移动3厘米
robot.set_body_pose([0.0, 3.0, 0.0], 600)
time.sleep(1)

# 设置身体在X轴负方向移动3厘米
robot.set_body_pose([-3.0, 0.0, 0.0], 600)
time.sleep(1)

# 设置身体在Y轴负方向移动3厘米
robot.set_body_pose([0.0, -3.0, 0.0], 600)
time.sleep(1)

# 设置身体在Z轴正方向移动3厘米（抬升高度）
robot.set_body_pose([0.0, 0.0, 3.0], 600)
time.sleep(1)

# 设置身体在Y轴负方向移动2厘米，Z轴负方向移动1厘米
robot.set_body_pose([0.0, -2.0, -1.0], 600)
time.sleep(1)

# 设置身体绕X轴旋转8度（Roll）
robot.set_body_angle([8.0, 0.0, 0.0], 600)
time.sleep(1)

# 设置身体绕X轴旋转-8度（反向Roll）
robot.set_body_angle([-8.0, 0.0, 0.0], 600)
time.sleep(1)

# 设置身体绕Y轴旋转12度（Pitch）
robot.set_body_angle([0.0, 12.0, 0.0], 600)
time.sleep(1)

# 设置身体绕Y轴旋转-12度（反向Pitch）
robot.set_body_angle([0.0, -12.0, 0.0], 600)
time.sleep(1)

# 设置身体绕Z轴旋转12度（Yaw）
robot.set_body_angle([0.0, 0.0, 12.0], 600)
time.sleep(1)

# 设置身体绕Z轴旋转-12度（反向Yaw）
robot.set_body_angle([0.0, 0.0, -12.0], 600)
time.sleep(1)

robot.reset()



