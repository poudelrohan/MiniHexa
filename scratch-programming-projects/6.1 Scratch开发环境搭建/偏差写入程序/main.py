'''
  MiniHexa——偏差设置程序（只需要设置一次，除非重新烧录固件）
'''


import Hiwonder  # 导入Hiwonder机器人控制库
import time      # 导入时间模块，用于延时控制

# 创建机器人对象
robot = Hiwonder.Robot()

# 设置舵机偏差值
robot.set_deviation(1 , 24)
robot.set_deviation(2 , 28)
robot.set_deviation(3 , 9)
robot.set_deviation(4 , -20)
robot.set_deviation(5 , -13)
robot.set_deviation(6 , -13)
robot.set_deviation(7 , 0)
robot.set_deviation(8 , -7)
robot.set_deviation(9 , 14)
robot.set_deviation(10 , 21)
robot.set_deviation(11 , -11)
robot.set_deviation(12 , 16)
robot.set_deviation(13 , 21)
robot.set_deviation(14 , 31)
robot.set_deviation(15 , -5)
robot.set_deviation(16 , 0)
robot.set_deviation(17 , -33)
robot.set_deviation(18 , -9)

time.sleep(0.5)
robot.download_deviation()
time.sleep(0.5)

print("******************************************")
print("servo deviation:")
print(robot.read_deviation())
print("******************************************")





