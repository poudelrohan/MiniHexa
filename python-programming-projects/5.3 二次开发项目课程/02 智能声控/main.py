import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
sound = Hiwonder.Sound()

while True:
  val = sound.read()
  if(val >20):
    robot.go([0.0, 2.0, 0.0] , 1 , 1000)
    time.sleep(1)
  time.sleep_ms(10)

