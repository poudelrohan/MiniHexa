import Hiwonder  # 导入Hiwonder机器人控制库
import time      # 导入时间模块，用于延时控制

# 创建机器人对象
robot = Hiwonder.Robot()

#执行动作组5.撒娇
robot.action_run(5)
time.sleep(4)


# 重置机器人到初始状态
robot.reset()