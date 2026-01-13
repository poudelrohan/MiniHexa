import Hiwonder  # 导入Hiwonder机器人控制库
import time      # 导入时间模块，用于延时控制


# 创建机器人对象
robot = Hiwonder.Robot()

# 沿Y轴正方向移动（前进）
robot.go([0, 3, 0], 3, 1000)
time.sleep(3)

# 沿X轴和Y轴正方向移动（右上45度方向）
robot.go([2, 2, 0], 3, 1000)
time.sleep(3)

# 沿X轴正方向移动（右移）
robot.go([3, 0, 0], 3, 1000)
time.sleep(3)

# 沿X轴正方向和Y轴负方向移动（右下45度方向）
robot.go([2, -2, 0], 3, 1000)
time.sleep(3)

# 沿Y轴负方向移动（后退）
robot.go([0, -3, 0], 3, 1000)
time.sleep(3)

# 沿X轴负方向和Y轴负方向移动（左下45度方向）
robot.go([-2, -2, 0], 3, 1000)
time.sleep(3)

# 沿X轴负方向移动（左移）
robot.go([-3, 0, 0], 3, 1000)
time.sleep(3)

# 沿X轴负方向和Y轴正方向移动（左上45度方向）
robot.go([-2, 2, 0], 3, 1000)
time.sleep(4)  # 等待4秒



