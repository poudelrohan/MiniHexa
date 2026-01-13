import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
ir1 = Hiwonder_DEV.DEV_IR(32)
ir2 = Hiwonder_DEV.DEV_IR(18)

# 主循环
while True:
    # 读取两个红外传感器的状态
    # 注意：True表示检测到障碍物，False表示无障碍物
    ir1_state = ir1.read()
    ir2_state = ir2.read()
    
    if ir1_state and ir2_state:  # 两个传感器都检测到障碍物
        # 后退2秒
        robot.go([0, -2, 0], 2, 800)
        time.sleep(2)  # 等待2秒
        
        # 顺时针旋转
        robot.go([0, 0, 2], 2, 800)
        time.sleep(2)  # 等待2秒
    
    elif ir1_state and not ir2_state:  # 只有传感器1检测到障碍物
        # 顺时针旋转
        robot.go([0, 0, 2], 2, 800)
        time.sleep(2)  # 等待2秒
    
    elif not ir1_state and ir2_state:  # 只有传感器2检测到障碍物
        # 逆时针旋转
        robot.go([0, 0, -2], 2, 800)
        time.sleep(2)  # 等待2秒
    
    else:  # 两个传感器都没有检测到障碍物
        # 前进（连续运动）
        robot.go([0, 2, 0], -1, 800)
    time.sleep(0.1)
