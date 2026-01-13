import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
ir1 = Hiwonder_DEV.DEV_IR(32)  # 红外传感器1连接到GPIO32
ir2 = Hiwonder_DEV.DEV_IR(18)  # 红外传感器2连接到GPIO18

# 主循环
while True:
    # 读取两个红外传感器的状态
    # 注意：True表示检测到障碍物，False表示无障碍物
    ir1_state = ir1.read()
    ir2_state = ir2.read()
    
    # 如果任一传感器检测到障碍物
    if ir1_state or ir2_state:
        # 后退动作：快速后退
        robot.go([0, -3, 0], 3, 1000)
        time.sleep(3)
        
        # 旋转
        robot.go([0, 0, 2], 4, 1000)
        time.sleep(4)
    
    # 如果两个传感器都没有检测到障碍物
    else:
        # 前进动作：连续前进
        robot.go([0, 3, 0])
    
    time.sleep(0.05)

