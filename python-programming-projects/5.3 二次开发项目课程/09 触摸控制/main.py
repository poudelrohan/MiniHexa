import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
touch = Hiwonder_DEV.DEV_TOUCH(32)

while True:
  if touch.read() == True:
    robot.go([0,1,0],2,1000)
    time.sleep(2)


