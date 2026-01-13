import Hiwonder
import Hiwonder_DEV
import time

Hiwonder.disableLowPowerAlarm()

robot = Hiwonder.Robot()
asr = Hiwonder_DEV.DEV_ASR()

FORWARD_ID = 1      # "前进"
BACKWARD_ID = 2     # "后退"
LEFT_ID = 3         # "左转"
RIGHT_ID = 4        # "右转"
STOP_ID = 9         # "停止"
WALK_ID = 29        # "走两步"

# 主循环
while True:
    # 执行语音识别
    result = asr.getResult()
    # 根据识别结果执行不同动作
    if result == FORWARD_ID:
        # 前进
        robot.go([0.0, 2.0, 0.0])
        
    elif result == BACKWARD_ID:
        # 后退
        robot.go([0.0, -2.0, 0.0])
        
    elif result == LEFT_ID:
        # 左转
        robot.go([0.0, 0.0, 2.0])
        
    elif result == RIGHT_ID:
        # 右转
        robot.go([0.0, 0.0, -2.0])
        
    elif result == STOP_ID:
        # 停止
        robot.go([0.0, 0.0, 0.0])
        
    elif result == WALK_ID:
        # "走两步" - 前进两步
        robot.go([0.0, 2.0, 0.0],2)
        time.sleep(2)
    
    time.sleep(0.1)




