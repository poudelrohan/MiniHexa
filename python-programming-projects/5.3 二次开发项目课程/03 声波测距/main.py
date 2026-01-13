import Hiwonder
import Hiwonder_DEV
import time

# 创建机器人对象和超声波传感器对象
robot = Hiwonder.Robot()
sonar = Hiwonder_DEV.DEV_SONAR()

# 初始化RGB灯为白色
sonar.setRGB(0, 255, 255, 255)  # 参数1: 灯的下标(0表示同时控制两个灯)
                                # 参数2-4: RGB值(255,255,255为白色)

def map_value(x, in_min, in_max, out_min, out_max):
    """将值从一个范围映射到另一个范围"""
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# 主循环
while True:
    # 读取超声波传感器距离（单位：厘米）
    distance = sonar.getDistance()
    
    # 根据距离设置RGB颜色
    if distance > 0 and distance <= 8:
        # 呼吸灯模式（红色） - 实际实现需要呼吸效果
        r, g, b = 255, 0, 0
    
    elif distance > 8 and distance <= 18:
        # 红色渐变（距离80-180映射到255-0）
        s = map_value(distance, 8, 18, 0, 255)
        r, g, b = 255 - s, 0, 0
    
    elif distance > 18 and distance <= 32:
        # 蓝色渐变（距离180-320映射到0-255）
        s = map_value(distance, 18, 32, 0, 255)
        r, g, b = 0, 0, s
    
    elif distance > 32 and distance <= 50:
        # 绿色渐变（距离320-500映射到0-255）
        s = map_value(distance, 32, 50, 0, 255)
        r, g, b = 0, s, 255 - s
    
    else:  # distance > 500
        # 绿色
        r, g, b = 0, 255, 0
    
    # 设置超声波传感器上的RGB灯
    sonar.setRGB(0, r, g, b)  # 同时设置两个灯
    print("Distance:", distance, "mm")
    time.sleep_ms(100)



