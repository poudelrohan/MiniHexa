import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()

led = Hiwonder_DEV.DEV_Digitaltube(32,14)

while True:
  led.drawStr(0,1,"abc")
  time.sleep(2)
  led.drawStr(0,1,"ABC")
  time.sleep(2)


