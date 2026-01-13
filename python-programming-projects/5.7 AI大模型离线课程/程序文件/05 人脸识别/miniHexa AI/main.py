import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
cam = Hiwonder_DEV.DEV_ESP32S3Cam()

position = [0, 0, 0]  # [x, y, z] 单位：厘米
initial_z = position[2]  # 保存初始高度

while True:
  rec = cam.read_face()
  if rec:
    if rec[2] > 0:
      # 俯仰+10度
      robot.set_body_angle([0, 10, 0], 300)
      time.sleep_ms(300)
      # 俯仰-10度
      robot.set_body_angle([0, -10, 0], 300)
      time.sleep_ms(300)
      # 俯仰+10度
      robot.set_body_angle([0, 10, 0], 300)
      time.sleep_ms(300)
      # 俯仰-10度
      robot.set_body_angle([0, -10, 0], 300)
      time.sleep_ms(300)
      # 回到水平姿态
      robot.set_body_angle([0, 0, 0], 300)
      time.sleep_ms(300)
      # 向右移动2cm
      robot.set_body_pose([2.0, 0, 0], 200)
      time.sleep_ms(200)
      # 向左移动2cm
      robot.set_body_pose([-2.0, 0, 0], 200)
      time.sleep_ms(200)
      # 向右移动2cm
      robot.set_body_pose([2.0, 0, 0], 200)
      time.sleep_ms(200)
      # 向左移动2cm
      robot.set_body_pose([-2.0, 0, 0], 200)
      time.sleep_ms(200)
      # 回到中心位置
      robot.set_body_pose([0, 0, 0], 200)
      time.sleep_ms(200)
  time.sleep(0.04)



