import Hiwonder
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
imu = Hiwonder.IMU()
time.sleep(1)

# 开启自平衡功能
robot.homeostasis(True)
time.sleep(3)

# 停止自平衡功能
# robot.homeostasis(False)

