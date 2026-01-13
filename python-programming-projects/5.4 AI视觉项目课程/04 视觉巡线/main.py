import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
cam = Hiwonder_DEV.DEV_ESP32S3Cam()

while True:
  rec = cam.read_color(1)
  if rec:
    if rec[0] > 120:
      robot.go([0,1,-0.1])
    elif rec[0] < 40:
      robot.go([0,1,0.1])
    else:
      robot.go([0,1,0])
  time.sleep(0.04)


