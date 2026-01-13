import Hiwonder  # 导入Hiwonder机器人控制库
import time      # 导入时间模块，用于延时控制


# 创建机器人对象
robot = Hiwonder.Robot()

# 沿Y轴正方向前进，同时进行逆时针旋转
robot.go([0, 3, 0.2])
time.sleep(5)

# 沿Y轴正方向前进，同时进行顺时针旋转
robot.go([0, 3, -0.2])
time.sleep(5)

# 重置机器人到初始状态
robot.reset()


