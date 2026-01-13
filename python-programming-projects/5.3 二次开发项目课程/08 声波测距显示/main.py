import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()

sonar = Hiwonder_DEV.DEV_SONAR()
led = Hiwonder_DEV.DEV_Digitaltube(32,33)

while True:
  distance = sonar.getDistance()
  distance = distance if distance < 500 else 500
  led.showNum(distance)
  time.sleep(0.2)



