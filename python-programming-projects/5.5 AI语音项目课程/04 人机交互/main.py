import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
sonar = Hiwonder_DEV.DEV_SONAR()
asr = Hiwonder_DEV.DEV_ASR()

def perform_dance():
    global robot
    # 第一部分：俯仰摆动（pitch摆动）
    # 俯仰+10度（身体前倾）
    robot.set_body_angle([0.0, 10.0, 0.0], 300)
    time.sleep_ms(600)  # 等待动作完成 + 额外延时
    
    # 俯仰-10度（身体后仰）
    robot.set_body_angle([0.0, -10.0, 0.0], 300)
    time.sleep_ms(600)
    
    # 俯仰+10度（身体前倾）
    robot.set_body_angle([0.0, 10.0, 0.0], 300)
    time.sleep_ms(600)
    
    # 俯仰-10度（身体后仰）
    robot.set_body_angle([0.0, -10.0, 0.0], 300)
    time.sleep_ms(600)
    
    # 回到水平姿态
    robot.set_body_angle([0.0, 0.0, 0.0], 300)
    time.sleep_ms(600)
    
    # 第二部分：水平摆动（x轴移动）
    # 向右移动2cm
    robot.set_body_pose([2.0, 0.0, 0], 200)
    time.sleep_ms(400)
    
    # 向左移动2cm
    robot.set_body_pose([-2.0, 0.0, 0], 200)
    time.sleep_ms(400)
    
    # 向右移动2cm
    robot.set_body_pose([2.0, 0.0, 0], 200)
    time.sleep_ms(400)
    
    # 向左移动2cm
    robot.set_body_pose([-2.0, 0.0, 0], 200)
    time.sleep_ms(400)
    
    # 回到中心位置
    robot.set_body_pose([0.0, 0.0, 0], 200)
    time.sleep_ms(400)

while True:
    result = asr.getResult()
    # 根据识别结果执行不同动作
    if result == 26:
        # 识别到"你好" - 执行动作组14
        robot.action_group_run(14)
        pass
    elif result == 27:
        # 识别到"介绍自己" - 执行卖萌动作
        perform_dance()
        
    elif result == 28:
        # 识别到"露一手" - 执行动作组7
        robot.action_group_run(7)
        pass
    time.sleep_ms(200)



