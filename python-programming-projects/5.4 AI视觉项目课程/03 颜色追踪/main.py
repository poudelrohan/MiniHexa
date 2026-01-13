import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
sonar = Hiwonder_DEV.DEV_SONAR()
cam = Hiwonder_DEV.DEV_ESP32S3Cam()

def fmap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

yaw = 0
# while True:
  # rec = cam.read_color(1)
  # if rec:
    # center = int(rec[0])
    # yaw += fmap(center , 0 , 160 , -1 , 1)
    # yaw = 20 if yaw > 20 else (-20 if yaw < -20 else yaw)
    # robot.set_body_angle([0,0,yaw],100)
  # time.sleep(0.05)



