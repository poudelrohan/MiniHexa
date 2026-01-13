import Hiwonder
import Hiwonder_DEV
import time

# 禁用低电量警报
Hiwonder.disableLowPowerAlarm()

# 创建机器人对象和超声波传感器对象
robot = Hiwonder.Robot()
sonar = Hiwonder_DEV.DEV_SONAR()

# 初始化RGB灯为白色
sonar.setRGB(0, 255, 255, 255)

# 主循环
while True:
    # 读取超声波传感器距离（单位：厘米）
    distance = sonar.getDistance()
    print(distance)
    if distance > 20:
      sonar.setRGB(0,0,250,0)
      robot.go([0,2,0])
    elif distance < 10:
      sonar.setRGB(0,250,0,0)
      robot.go([0,-2,0])
    else:
      sonar.setRGB(0,0,0,250)
      robot.go([0,0,0])
    time.sleep_ms(100)



