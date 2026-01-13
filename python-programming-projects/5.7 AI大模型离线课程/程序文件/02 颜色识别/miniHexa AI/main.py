import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
sonar = Hiwonder_DEV.DEV_SONAR()
cam = Hiwonder_DEV.DEV_ESP32S3Cam()

while True:
  rec = cam.read_color(1)
  if rec:
    print("red")
    sonar.setRGB(0,250,0,0)
  else:
    rec = cam.read_color(2)
    if rec:
      print("green")
      sonar.setRGB(0,0,250,0)
    else:
      rec = cam.read_color(3)
      if rec:
        print("blue")
        sonar.setRGB(0,250,0,0)
      else:
        sonar.setRGB(0,0,0,0)
  time.sleep(0.2)

