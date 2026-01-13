import Hiwonder  # 导入Hiwonder机器人控制库
import time      # 导入时间模块，用于延时控制


# 创建机器人对象
robot = Hiwonder.Robot()

# 前进运动
robot.go([0, 2, 0], 3, 600)
time.sleep(4)

# 前进运动
robot.go([0, 2, 0], 2, 1000)
time.sleep(4)

# 后退运动
robot.go([0, -2, 0], 3, 600)
time.sleep(4)

# 后退运动
robot.go([0, -2, 0], 2, 1000)
time.sleep(4)

# 顺时针旋转
robot.go([0, 0, -2], 3, 600)
time.sleep(4)

# 顺时针旋转
robot.go([0, 0, -2], 2, 1000)
time.sleep(4)

# 重置机器人到初始状态
robot.reset()


