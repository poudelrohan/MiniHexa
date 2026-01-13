import Hiwonder  # 导入Hiwonder机器人控制库
import time      # 导入时间模块，用于延时控制


# 创建机器人对象
robot = Hiwonder.Robot()

# 第一次旋转：低速旋转（1单位速度）
robot.go([0, 0, 1])
time.sleep(3)

# 第二次旋转：中低速旋转（1.5单位速度）
robot.go([0, 0, 1.5])
time.sleep(3)

# 第三次旋转：中速旋转（2单位速度）
robot.go([0, 0, 2])
time.sleep(3)

# 第四次旋转：高速旋转（2.5单位速度）
robot.go([0, 0, 2.5])
time.sleep(3)

# 重置机器人到初始状态
robot.reset()


