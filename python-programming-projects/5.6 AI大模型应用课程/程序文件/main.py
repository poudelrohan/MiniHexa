import gc
import heapq
import json
import struct
import _thread
import time      # 导入时间模块，用于延时控制
import Hiwonder  # 导入Hiwonder机器人控制库
import Hiwonder_DEV
from machine import ADC

Move_stride_LowSpeed_1 = 70       #低速模式下前后运动单步平移步幅（单位：毫米）
Move_stride_HighSpeed_1 = 85      #高速模式下前后运动单步平移步幅（单位：毫米）
Move_stride_LowSpeed_2 = 65       #低速模式下左右运动单步平移步幅（单位：毫米）
Move_stride_HighSpeed_2 = 75      #高速模式下左右运动单步平移步幅（单位：毫米）
Move_stride_LowSpeed_3 = 90       #低速模式下斜向运动单步平移步幅（单位：毫米）
Move_stride_HighSpeed_3 = 90      #高速模式下斜向运动单步平移步幅（单位：毫米）
Rotation_angle_LowSpeed = 24      #低速模式下单步旋转角度（单位：度）
Rotation_angle_HighSpeed = 45     #高速模式下单步旋转角度（单位：度）

# 创建机器人对象
robot = Hiwonder.Robot()
beep = Hiwonder.Buzzer()
imu = Hiwonder.IMU()
i2csonar = Hiwonder_DEV.DEV_SONAR()
adc = ADC(33,ADC.ATTN_6DB)        #初始化ADC，6DB衰减控制测量电压范围
adc.width(ADC.WIDTH_12BIT)

vel = [0,0,0] #机器人在xyz三轴上的运动速度
pos = [0,0,0] #机器人的重心在坐标系下的三维坐标
att = [0,0,0] #机器人绕xyz三轴旋转的欧拉角

euler = [0,0,0] #机器人三轴欧拉角

motion_target_step = 0        #常规运动下的运动步数
motion_target_RunningTime = 0 #常规运动下的运动持续时间
motion_target_RotationAngle = 0 #常规运动下的转动角度(仅用于原地左右转)
motion_target_distance = 0    #常规运动下的运动距离(用于八个方向的平移)
move_direction = 1		      #常规运动下的机器人平移移动方向，1-正前、2-右前、3-正右、4-右后、5-正后、6-左后、7-正左、8-左前 9-原地左转 10-原地右转
running_mode = 1              #1-normal(常规), 2-intelligent_avoidance(智能避障), 3-intelligent_track(智能跟随)
avoid_distance = 0         	  #机器人与目标保持的最小距离间隔，用于智能避障、智能跟随模式
rgb_left = [0,0,0]            #发光超声波模块左侧RGB灯光强比
rgb_right = [0,0,0]           #发光超声波模块右侧RGB灯光强比
buzzer_count = 0              #蜂鸣器鸣响次数
Speed_Mode = 1            	  #1-Low_speed(低速), 2-BelowMiddle_speed(中低速), 3-Medium_speed(中速), 4-High_speed(高速)
Detection_WonderMind = 0	  #0-总线未检测到模块  1-总线检测到模块
Vision_Result = 0        	  #视觉识别结果
incline_direction = 0    	  #常规运动下的机器人绕欧拉角转动倾斜方向，1-前倾、2-后倾、3-左倾、4-右倾、5-左扭、6-右扭
BaryCenter_direction = 0  	  #常规运动下的机器人重心移动方向，1-正前、2-右前、3-正右、4-右后、5-正后、6-左后、7-正左、8-左前、9-正上、10-正下
ActionNum = 0                 #执行动作组序号
ExecuteActionNum = 0          #执行动作组次数

sustain_move_flag = 0         #机器人持续运动标志位
timer = 0

func_follow_num = 0            #follow函数专用计数器
func_avoid_num = 0             #avoid函数专用计数器

class Controller:
    sleep_command_dict = {
      "command":     "sleep",
      "params":      "true",
    }

    abort_command_dict = {
      "command":     "abort",
      "params":      "true",
    }  

    vision_command_dict = {
      "command":     "vision",
      "params":      "text",
    }

    voice_command_dict = {
      "command":     "voice",
      "params":      "text",
    }


    status_command_dict = {
      "command":     "status",
      "params":      [],
    }

    action_finish_command_dict = {
      "command":     "action_finish",
      "params":      "true",
    }
    mcp_finish_setting_dict = {
      "command":     "mcp_setting",
      "params":      "params",
    } 

    status = [
        "unknown",
        "starting",
        "configuring",
        "idle",
        "connecting",
        "listening",
        "speaking",
        "upgrading",
        "activating",
        "audio_testing",
        "fatal_error",
        "invalid_state"
    ]


    def __init__(self, i2c):
        self.addr = 0x55 
        self.i2c = i2c
        self.data_heap = []  
        self.counter = 0    

        # 接收队列 (priority, count, data)
        self.recv_heap = [] # 用于存储接收到的数据的优先队列，元素为 (priority, data)
        self.recv_counter = 0  # 用于保证相同优先级数据的顺序
        # 发送队列 (priority, count, dict_data)
        self.send_heap = []
        self.send_counter = 0      
        self.running = True

        # Add heap size limits to prevent unbounded growth
        self.max_heap_size = 10

        self.lock = _thread.allocate_lock()


        # 分片数据重组相关
        self.fragments = {}  # 存储分片数据，格式为 {fragment_id: [data, total_fragments, received_fragments]}
        self.fragment_timeout = 2.0  # 分片超时时间（秒）
        self.last_fragment_time = 0  # 上次接收分片的时间

        while True:
          devices = self.i2c.scan()
          addr = [hex(d) for d in devices]
          if '0x55' in addr:
            break
          time.sleep(0.01)   
        time.sleep(0.1)   

    def enqueue_send(self, dict_data, priority=0):
        """把要发的数据丢进发送队列"""
        # Prevent heap from growing too large
        with self.lock:
            if len(self.send_heap) >= self.max_heap_size:
                print("Send heap full, discarding oldest message")
                heapq.heappop(self.send_heap)
            heapq.heappush(self.send_heap, (priority, self.send_counter, dict_data))
            self.send_counter += 1         

    def start(self):    
      time.sleep(0.1)
      self.send_message(self.mcp_finish_setting_dict) 
      _thread.start_new_thread(self.callback, ())

    def stop(self):    
      self.running = False

    def sleep(self):
      self.enqueue_send(self.sleep_command_dict)


    def abort(self):
      self.send_message(self.abort_command_dict)  

    def vision(self, text):
      vision_cmd = self.vision_command_dict.copy()
      vision_cmd['params'] = text
      self.send_message(vision_cmd) 

    def tts(self, text):
      tts_cmd = self.tts_command_dict.copy()
      tts_cmd['params'] = text
      self.send_message(tts_cmd) 

    def set_tts_model(self, model):
      tts_model_cmd = self.tts_model_command_dict.copy()
      tts_model_cmd['params'] = model
      self.send_message(tts_model_cmd) 

    def set_voice(self, voice):
      voice_cmd = self.voice_command_dict.copy()
      voice_cmd['params'] = self.voice[voice]
      self.send_message(voice_cmd) 

    def send_status(self, data):
      status_cmd = self.status_command_dict.copy()
      status_cmd['params'] = data
      self.send_message(status_cmd)

    def send_action_finish(self, data):
      action_cmd = self.action_finish_command_dict.copy()
      action_cmd['params'] = data
      self.send_message(action_cmd)



    def process_fragment(self, fragment_id, total_fragments, actual_data):

        """处理分片数据，当所有分片都接收完成后重组数据"""

        current_time = time.time()



        # print(f"Processing fragment: ID={fragment_id}, total={total_fragments}, data_len={len(actual_data)}")

        # 检查是否超时，如果超时则清空分片缓存
        if self.last_fragment_time > 0 and current_time - self.last_fragment_time > self.fragment_timeout:
            self.fragments.clear()
            print("Fragment timeout, cleared all fragments")

        self.last_fragment_time = current_time

        # 如果是单分片数据，直接返回
        if total_fragments == 1:
            # print("Single fragment data, returning directly")

            return actual_data



        # 如果是多分片数据，需要重组

        # 使用当前时间戳作为分片组的标识，这样可以区分不同的数据组

        # 只有第一个分片（fragment_id == 1）会创建新的分片组
        if fragment_id == 1:
            # 创建新的分片组，使用当前时间戳作为键
            group_id = int(current_time * 1000)  # 使用毫秒级时间戳
            self.fragments[group_id] = {
                'data': [None] * total_fragments,
                'total_fragments': total_fragments,
                'received_fragments': 0,
                'start_time': current_time
            }
            # print(f"Created new fragment group {group_id} with {total_fragments} fragments")

        # 如果没有分片组，说明我们错过了第一个分片，忽略当前分片
        if not self.fragments:
            print("No fragment groups available, possibly missed first fragment")
            return None

        # 获取最新的分片组（即最近创建的分片组）
        group_id = max(self.fragments.keys())
        fragment_info = self.fragments[group_id]

        # print(f"Using fragment group {group_id}, received {fragment_info['received_fragments']}/{fragment_info['total_fragments']} fragments")



        # 检查分片ID是否有效

        # 分片ID从1开始，所以需要减1作为数组索引

        if fragment_id >= 1 and fragment_id <= total_fragments:
            # 分片数据，转换为0-based索引
            fragment_index = fragment_id - 1
        else:
            print(f"Invalid fragment ID: {fragment_id}, total fragments: {total_fragments}")
            return None

        # 检查当前分片组的总分片数是否匹配
        if fragment_info['total_fragments'] != total_fragments:
            print(f"Fragment count mismatch: expected {fragment_info['total_fragments']}, got {total_fragments}")
            return None



        # 只有分片数据（fragment_id >= 1）才需要存储

        if fragment_id >= 1:

            # 存储分片数据

            if fragment_info['data'][fragment_index] is None:

                fragment_info['data'][fragment_index] = actual_data
                fragment_info['received_fragments'] += 1
                # print(f"Stored fragment {fragment_id} (index {fragment_index}), now have {fragment_info['received_fragments']}/{total_fragments}")
            else:
                print(f"Fragment {fragment_id} (index {fragment_index}) already received, ignoring duplicate")

            # 检查是否所有分片都已接收
            if fragment_info['received_fragments'] == total_fragments:
                # print("All fragments received, reassembling data")
                # 重组数据
                reassembled_data = b''.join(fragment_info['data'])
                # 从分片缓存中移除
                del self.fragments[group_id]
                # print(f"Reassembled data length: {len(reassembled_data)}")
                return reassembled_data

        return None

    def calculate_checksum(self, data):
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum & 0xFF

    def send_message(self, dict_data):
        try:
            data = json.dumps(dict_data).encode("utf-8")
            # print('data len: %d, max data len:1024\n'%len(data))
            if len(data) > 1024:
                print("Warning: Data too large, truncating")
                return False
            self.i2c.writeto(self.addr, data)
            time.sleep(0.05)
            return True
        except Exception as e:
            print(f"Send error: {e}")
            return False

    def get_message(self):
        """从优先队列中取出一个数据，如果队列空返回None"""
        with self.lock:
            if self.recv_heap:
                #print("2\r\n")
                priority, count, data = heapq.heappop(self.recv_heap)
                # data = json.dumps(data, ensure_ascii=False)
                return data
            else:
                return None

    def callback(self):
        gc_counter = 0  # Add counter for periodic garbage collection

        while self.running:
            try:
                # Periodic garbage collection every 100 iterations
                gc_counter += 1
                if gc_counter % 100 == 0:
                    gc.collect()
                    # print(f"GC: Free memory after collection")

                # 发送，如果有的话
                with self.lock:
                    while self.send_heap:
                        _, _, msg = heapq.heappop(self.send_heap)
                        try:
                            raw = json.dumps(msg).encode("utf-8")
                            self.i2c.writeto(self.addr, raw)
                        except Exception as e:
                            print(f"Send error in callback: {e}")
                            break  # Exit send loop on error

                # Limit receive heap size
                with self.lock:
                    if len(self.recv_heap) >= self.max_heap_size:
                        #print("1\r\n")
                        heapq.heappop(self.recv_heap)



                # 读取头部信息：AA55（帧头）+ 数据长度（2字节）+ 分片ID（2字节）+ 总分片数（2字节）

                try:

                    # 添加延时，确保从机已经准备好发送数据
                    time.sleep(0.01)
                    header_data = self.i2c.readfrom(self.addr, 8)
                    # print(f"Header data: {header_data.hex()}")

                    # 检查头部数据是否有效
                    if len(header_data) != 8:
                        print(f"Invalid header length: {len(header_data)}, expected 8")
                        time.sleep(0.1)
                        continue

                    # 检查帧头是否有效
                    flag = struct.unpack('>H', header_data[0:2])[0]
                    if flag != 0xAA55:
                        print(f"Invalid flag: 0x{flag:04X}, expected 0xAA55")
                        time.sleep(0.1)
                        continue
                except Exception as e:
                    print(f"Error reading header: {e}")
                    time.sleep(0.1)
                    continue

                data_len = struct.unpack('>H', header_data[2:4])[0]
                fragment_id = struct.unpack('<H', header_data[4:6])[0]
                total_fragments = struct.unpack('<H', header_data[6:8])[0]

                # print(f"Received: flag=0x{flag:04X}, data_len={data_len}, fragment_id={fragment_id}, total_fragments={total_fragments}")

                if data_len > 0:
                    # Add size limit check
                    if data_len > 8192:  # Reasonable limit
                        print(f"Data too large: {data_len} bytes, skipping")
                        continue

                    try:
                        # 添加延时，确保从机已经准备好发送数据
                        time.sleep(0.01)
                        data_with_checksum = self.i2c.readfrom(self.addr, data_len + 1)

                        # 检查数据长度是否正确
                        if len(data_with_checksum) != data_len + 1:
                            print(f"Invalid data length: {len(data_with_checksum)}, expected {data_len + 1}")
                            time.sleep(0.1)
                            continue

                        actual_data = data_with_checksum[:-1]
                        received_checksum = data_with_checksum[-1]
                        calculated_checksum = self.calculate_checksum(actual_data)

                        # print(f"Data: {actual_data.hex()}, received_checksum=0x{received_checksum:02X}, calculated_checksum=0x{calculated_checksum:02X}")
                    except Exception as e:
                        print(f"Error reading data: {e}")
                        time.sleep(0.1)
                        continue

                    if received_checksum == calculated_checksum:
                        # 处理分片数据
                        reassembled_data = self.process_fragment(fragment_id, total_fragments, actual_data)

                        # 如果数据重组完成（对于单分片数据，会立即返回）
                        if reassembled_data is not None:
                            if len(reassembled_data) == 1:
                                status_idx = struct.unpack('B', reassembled_data)[0]
                                # print(f"Status index: {status_idx}")
                                if status_idx < len(self.status):
                                    data = self.status[status_idx]
                                else:
                                    print(f"Invalid status index: {status_idx}")
                                    continue
                            else:
                                try:
                                    data = json.loads(reassembled_data.decode('utf-8'))
                                    # print(f"JSON data: {data}")
                                except Exception as e:
                                    print(f"JSON decode error: {e}")

                                    continue



                            # 这里给数据一个优先级，假设所有数据优先级相同为0

                            # 你可以根据data内容自定义优先级

                            # 使用counter保证相同优先级数据的顺序
                            priority = 0
                            with self.lock:
                                #print(data)
                                heapq.heappush(self.recv_heap, (priority, self.recv_counter, data))
                                self.recv_counter += 1
                    else:
                        print(f"Checksum error: received=0x{received_checksum:02X}, calculated=0x{calculated_checksum:02X}")

            except Exception as e:
                print("Callback error:", e)
                # Add a longer sleep on error to prevent rapid error loops
                time.sleep(0.5)
                continue

            time.sleep(0.02)

# 使用示例

#设置mcp，调用mcp时会触发从机返回params格式数据回来，其中string会由大模型给出，跟你跟大模型的对话决定，例如你要求获取当前姿态，那么他就可能返回的是pose
#注意command里要描述清楚有哪些返回的选项，return表示是否需要返回数据给从机，如果需要，那么返回的数据按send_status的要求给出，他是一个列表

#如[["pose", "0"], ["battery", 8000]], 从机接收到后会原样返回给大模型让他进行判断或者播报

tool_buzzer = {
	"tool_name": "self.robot.set_buzzer",
	"command": "控制机器人的蜂鸣器时调用这个工具。'count'是蜂鸣器响的次数",
	"params":[["count","int"]],
	"block": "true",
	"return": "false",
}

tool_led = {
	"tool_name":"self.robot.set_led_color",
	"command":"设置左右RGB灯颜色。lr,lg,lb是左灯RGB, rr,rg,rb是右灯RGB, 范围0-255。",
	"params":[["lr","int","0","255"],["lg","int","0","255"],["lb","int","0","255"],["rr","int","0","255"],["rg","int","0","255"],["rb","int","0","255"]],
	"block":"true",
	"return":"false",
}

tool_SpeedMode = {
	"tool_name": "self.robot.set_SpeedMode",
	"command": "切换机器人的速度模式时调用这个工具。可切换的速度模式从1到2档包括：'Low_speed', 'High_speed'。",
	"params":[["speed_mode","string"]],
	"block":"true",
	"return":"false",
}

tool_RunningMode = {
	"tool_name":"self.robot.set_RunningMode",
	"command":"切换机器人的运动模式时调用这个工具。distance为距离,默认为200,单位毫米,可切换的运动模式包括：'normal','intelligent_avoidance', 'intelligent_track'。",
	"params":[["running_mode","string"],["distance","int"]],
	"block":"true",
	"return":"false",
}

tool_ActionGroup = {
	"tool_name":"self.robot.ActionGroup",
	"command":"控制机器人执行动作组时调用这个工具。actionNum为动作组代号,3号为唤醒,4号为唤醒奔跑,5号为撒娇,6号为越障,7号为左腿战斗,8号为右腿战斗,9号为左脚向前踢球,10号为左脚向右踢球,11号为右脚向前踢球,12号为右脚向左踢球,13号为推门,14号为挥手,executeNum为动作组运行次数",
	"params":[["actionNum","int","3","14"],["executeNum","int"]],
	"block":"true",
	"return":"false",
}

tool_status = {
	"tool_name":"self.robot.get_status",
	"command":"获取机器人的实时状态时调用这个工具。可查询的状态包括：'battery', 'distance', 'poseture','running_mode','Speed_Mode'。",
	"params":[["status_name","string"]],
	"block":"true",
	"return":"true",
}

tool_move = {
	"tool_name":"self.robot.move",
	"command":"控制机器人平移时调用。move为运动方向,分别为前、右前、右、右后、后、左后、左、左前, 原地左转, 原地右转,分别使用代号1至10。step_num是运动步数默认为-1。duration是运动时长默认-1毫秒。distance是运动距离默认为-1厘米。angle参数是运动角度默认为-1度。",
	"params":[["move","int","1","10"],["step_num","int"],["duration","int"],["distance","int"],["angle","int"]],
	"block":"true",
	"return":"false",
}

tool_BaryCenterMove = {
	"tool_name":"self.robot.BaryCenterMove",
	"command":"控制机器人重心移动时调用这个工具。pose参数控制运动方向,分别为正前、右前、正右、右后、正后、左后、正左、左前、正上、正下,分别使用代号1至10,如需恢复初始姿态使用11。",
	"params":[["pose","int","1","11"]],
	"block":"true",
	"return":"false",
}

tool_InclinationAngleMove = {
	"tool_name":"self.robot.InclinationAngleMove",
	"command":"控制机器人姿态倾斜时调用这个工具。Inclination参数控制运动方向,分别为前倾、后倾、左倾、右倾、左扭、右扭,分别使用代号1至6。",
	"params":[["Inclination","int","1","6"]],
	"block":"true",
	"return":"false",
}


time.sleep(1)
control = Controller(Hiwonder_DEV.IIC())
control.send_message(tool_buzzer)
control.send_message(tool_led)
control.send_message(tool_SpeedMode) 
control.send_message(tool_RunningMode)
control.send_message(tool_ActionGroup)
control.send_message(tool_status)
control.send_message(tool_move)
control.send_message(tool_BaryCenterMove)
control.send_message(tool_InclinationAngleMove)

control.start()

i2csonar.setRGB(0,0,0,255)
beep.setVolume(100)
beep.playTone(1500, 200, True)

'''电压获取接口'''
def battery_Voltage_detect():
  read_Voltage = adc.read()   #读取模拟电平，返回值范围0-4095
  read_Voltage *= 3.3/4096  #将ADC电压值转换为模拟电压（分压电阻上的分到的部分电源电压）
  read_Voltage *= 11 #分压电阻分到原始电压的1/11
  return read_Voltage

'''执行唤醒奔跑动作'''
def wake_up():
  vel = [0,0,0]
  robot.go(vel, 0, 0)
  pos = [0,0,0]
  robot.set_body_pose(pos, 200)
  att = [0,0,0]
  robot.set_body_angle(att, 200)

  time.sleep(0.1)
  att = [-8,0,-15]
  robot.set_body_angle(att, 200)

  time.sleep(1)
  att = [-8,0,15]
  robot.set_body_angle(att, 200)
  
  time.sleep(1)
  att = [0,0,0]
  robot.set_body_angle(att, 200)

  time.sleep(0.6)
  vel = [0,1.5,0]
  robot.go(vel, 10, 600)
  time.sleep(6)
  
'''执行唤醒动作'''
def _wake_up():
  vel = [0,0,0]
  robot.go(vel, 0, 0)
  pos = [0,0,0]
  robot.set_body_pose(pos, 200)
  att = [0,0,0]
  robot.set_body_angle(att, 200)

  att = [-8,0,-15]
  robot.set_body_angle(att, 200)
  time.sleep(0.1)
  
  att = [-8,0,15]
  robot.set_body_angle(att, 600)
  time.sleep(0.1)
  
  att = [0,0,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.2)
  
'''执行撒娇动作'''
def acting_cute():
  vel = [0,0,0]
  robot.go(vel, 0, 0)
  pos = [0,0,0]
  robot.set_body_pose(pos, 200)
  att = [0,0,0]
  robot.set_body_angle(att, 200)
  
  att = [0,10,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  att = [0,-10,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  att = [0,10,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  att = [0,-10,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  att = [0,0,0]
  robot.set_body_angle(att, 200)
  time.sleep(0.3)
  
  pos = [2,0,0]
  robot.set_body_pose(pos, 200) 
  time.sleep(0.2)
  
  pos = [-2,0,0]  
  robot.set_body_pose(pos, 200) 
  time.sleep(0.2)
  
  pos = [2,0,0]
  robot.set_body_pose(pos, 200) 
  time.sleep(0.2)
  
  pos = [-2,0,0]  
  robot.set_body_pose(pos, 200) 
  time.sleep(0.2)
  
  pos = [0,0,0]
  robot.set_body_pose(pos, 200)  
  time.sleep(0.2)

'''dis_avoid ： 目标间距'''
def avoid(dis_avoid):
  global func_avoid_num #声明外部变量
  
  func_avoid_num +=1
  #每5*0.02=0.1s 执行1次内部逻辑，避免过高频率读取超声波模块读取到异常数据
  if(func_avoid_num % 5 == 0):
    _distance = i2csonar.getDistance() *10 #cm--->mm
    print(_distance)
    if (_distance<=100): #低于临界距离，控制机器人后退
      robot.go([0,-2,0],4,1000)
      time.sleep(4)
    else:
      if (_distance<=dis_avoid): #低于目标间距但大于临界距离，控制机器人转弯
        robot.go([0,0,1.8],4,1000)
        time.sleep(4)
      else: #大于目标间距，控制机器人前进
        robot.go([0,2,0],-1,800)

'''dis_avoid ： 目标间距'''
def follow(dis_avoid):
  global func_follow_num #声明外部变量
  
  func_follow_num +=1
  #每5*0.02=0.1s 执行1次内部逻辑，避免过高频率读取超声波模块读取到异常数据
  if(func_follow_num % 5 ==0):
    _distance = i2csonar.getDistance() *10
    print(_distance)
    if (_distance<=dis_avoid -50): #低于临界距离，控制机器人后退
      robot.go([0,-2,0],-1,500)
    else:
      if (_distance>=dis_avoid +50): #大于目标间距，控制机器人前进
        robot.go([0,2,0],-1,500)
      else:  #低于目标间距但大于临界距离，控制机器人停下
        robot.go([0,0,0],0)


# ================== 主循环 ==================
robot_status = []
loop_counter = 0

try:
    while True:
        loop_counter += 1
        if loop_counter % 1000 == 0:
            gc.collect()

        data = control.get_message()
        if data:
            if data not in control.status:
                print("Received data from queue:", data)

                try:
                    if 'move' in data:          #tool_move工具命令
                        move_direction = data.get('move', 0)
                        motion_target_step = data.get('step_num', 0)
                        motion_target_RunningTime = data.get('duration', 0)
                        motion_target_RotationAngle = data.get('angle', 0)
                        motion_target_distance = data.get('distance', 0)
					
                        if(motion_target_RotationAngle == -1 and motion_target_distance == -1): #不指定运动距离和运动角度
          
                          #不指定步数或运动时长，默认一直走不停下
                          if(motion_target_step == -1 and motion_target_RunningTime == -1): 
                            sustain_move_flag = 1

                          #指定运动步数但不指定运动时长
                          elif(motion_target_step != -1 and motion_target_RunningTime == -1): 
                            timer = 600 * motion_target_step #迈一步默认耗时600ms

                          #指定运动步数但不指定运动时长
                          elif(motion_target_step == -1 and motion_target_RunningTime != -1):
                            timer = motion_target_RunningTime

                          #同时指定运动步数和运动时长，两个参数相冲突，以运动时长为准
                          elif(motion_target_step != -1 and motion_target_RunningTime != -1): 
                            timer = motion_target_RunningTime
                            
                        elif(motion_target_RotationAngle != -1): #指定运动角度
    
                          #将运动角度换算为运动步数，按照 “不指定运动距离和运动角度” -- “指定运动步数但不指定运动时长” 处理
                          if(Speed_Mode == 1):  #低速模式
                            motion_target_step = motion_target_RotationAngle / Rotation_angle_LowSpeed

                          elif(Speed_Mode == 2): #高速模式
                            motion_target_step = motion_target_RotationAngle / Rotation_angle_HighSpeed
            
                          timer = 600 * motion_target_step  #迈一步默认耗时600ms
                          
                        elif(motion_target_distance != -1):      #指定运动距离

                          #将运动距离换算为运动步数，按照 “不指定运动距离和运动角度” -- “指定运动步数但不指定运动时长” 处理
                          if(Speed_Mode == 1):  #低速模式
                            if(move_direction == 1 or move_direction == 5): #前后
                              motion_target_step = motion_target_distance / Move_stride_LowSpeed_1
                              
                            elif(move_direction == 3 or move_direction == 7): #左右
                              motion_target_step = motion_target_distance / Move_stride_LowSpeed_2
 
                            elif(move_direction == 2 or move_direction == 4 or move_direction == 6 or move_direction == 8): #斜向
                              motion_target_step = motion_target_distance / Move_stride_LowSpeed_3

                          elif(Speed_Mode == 2): #高速模式
                            if(move_direction == 1 or move_direction == 5): #前后
                              motion_target_step = motion_target_distance / Move_stride_HighSpeed_1
                              
                            elif(move_direction == 3 or move_direction == 7): #左右
                              motion_target_step = motion_target_distance / Move_stride_HighSpeed_2
 
                            elif(move_direction == 2 or move_direction == 4 or move_direction == 6 or move_direction == 8): #斜向
                              motion_target_step = motion_target_distance / Move_stride_HighSpeed_3

                          timer = 600 * motion_target_step #迈一步默认耗时600ms
    
                        else:                                    #同时指定运动距离和运动角度,两个参数冲突，机器人不执行本轮指令
                        
                          motion_target_step = 0
                          motion_target_RunningTime = 0
                          
                          
                        if (move_direction == 1):   #1-正前
                          if(motion_target_step == 0 and motion_target_RunningTime == 0): #当前指令为“机器人停止运动”
                            sustain_move_flag = 0
             
                          vel = [0,2*Speed_Mode,0]
							
                        elif (move_direction == 2): #2-右前
                          vel = [2*Speed_Mode,2*Speed_Mode,0]
							
                        elif (move_direction == 3): #3-正右
                          vel = [2*Speed_Mode,0,0]
							
                        elif (move_direction == 4): #4-右后
                          vel = [2*Speed_Mode,-2*Speed_Mode,0]
							
                        elif (move_direction == 5): #5-正后
                          vel = [0,-2*Speed_Mode,0]
							
                        elif (move_direction == 6):	#6-左后
                          vel = [-2*Speed_Mode,-2*Speed_Mode,0]
							
                        elif (move_direction == 7):	#7-正左					
                          vel = [-2*Speed_Mode,0,0]
							
                        elif (move_direction == 8): #8-左前
                          vel = [-2*Speed_Mode,2*Speed_Mode,0]
							
                        elif (move_direction == 9): #9-原地左转
                          vel = [0,0,2*Speed_Mode]
							
                        elif (move_direction == 10):#10-原地右转	
                          vel = [0,0,-2*Speed_Mode]
							
                        else:
                          vel = [0,0,0]
							
                        robot.go(vel, motion_target_step, 600)
						
                        if(sustain_move_flag != 1):
                          time.sleep(timer/1000) #这里的单位是s，因此/1000作单位转换
                          vel = [0,0,0]
                          robot.go(vel, 0, 0)
          
                        control.send_action_finish("true")

                    if 'Inclination' in data:   #tool_InclinationAngleMove工具命令
                        incline_direction = data.get('Inclination', 0)
                        
                        if (incline_direction == 1):   #1-前倾
                          att = [12,0,0]
							
                        elif (incline_direction == 2):   #2-后倾
                          att = [-12,0,0]
							
                        elif (incline_direction == 3):   #3-左倾
                          att = [0,15,0]
							
                        elif (incline_direction == 4):   #4-右倾
                          att = [0,-15,0]
							
                        elif (incline_direction == 5):   #5-左扭
                          att = [0,0,-15]
							
                        elif (incline_direction == 6):   #6-右扭
                          att = [0,0,15]
							
                        else:
                          att = [0,0,0]
							
                        robot.set_body_angle(att, 600)
                        control.send_action_finish("true")

                    '''注意：get_status类型消息检测必须在set_mode类型消息类检测前进行，
                      否则获取运动状态的get_status消息帧因为含running_mode字段
                      也会被误判成set_mode类型消息'''
                    '''get_status类型消息检测必须在BaryCenterMove类型消息类检测前进行，原因同上'''
                    if 'status_name' in data:   #tool_status工具命令
                      if data['status_name'] == 'battery':
                        value = Hiwonder.Battery_power()
                        print(f"Battery: {value}")
                        robot_status.append(["battery", str(value),"mv"])

                      if data['status_name'] == 'Speed_Mode':
                        robot_status.append(["Speed_mode", str(Speed_Mode)])

                      if data['status_name'] == 'distance':
                        value = i2csonar.getDistance() *10  #函数返回的单位是cm，单位转换为mm
                        robot_status.append(["distance", str(value),"mm"])

                      if data['status_name'] == 'running_mode':
                        robot_status.append(["running_mode", str(running_mode)])
	
                      if data['status_name'] == 'poseture':
                        euler = imu.read_angle()
                        robot_status.append([["roll", str(euler[0])],["pitch", str(euler[1])]])
  
                      if robot_status:
                        control.send_status(robot_status)
                        robot_status.clear()
							
                    if 'pose' in data:          #tool_BaryCenterMove工具命令
                      BaryCenter_direction = data.get('pose', 0)
                        
                      if (BaryCenter_direction == 1):   #1-正前
                        pos = [0,3,0]
							
                      elif (BaryCenter_direction == 2):   #2-右前
                        pos = [3,3,0]
							
                      elif (BaryCenter_direction == 3):   #3-正右
                        pos = [3,0,0]
							
                      elif (BaryCenter_direction == 4):   #4-右后
                        pos = [3,-3,0]
							
                      elif (BaryCenter_direction == 5):   #5-正后
                        pos = [0,-3,0]
							
                      elif (BaryCenter_direction == 6):   #6-左后
                        pos = [-3,-3,0]
							
                      elif (BaryCenter_direction == 7):   #7-正左
                        pos = [-3,0,0]
							
                      elif (BaryCenter_direction == 8):   #8-左前
                        pos = [-3,3,0]
							
                      elif (BaryCenter_direction == 9):   #9-正上
                        pos = [0,0,3]
							
                      elif (BaryCenter_direction == 10):  #10-正下	
                        pos = [0,0,-1.5]
							
                      else: #11-回复初始姿态
                        robot.set_body_angle([0,0,0],600)
                        pos = [0,0,0]
							
                      robot.set_body_pose(pos, 600)
                      control.send_action_finish("true")
              
                    if 'running_mode' in data:  #tool_RunningMode工具命令
                      if data['running_mode'] == 'normal':
                        running_mode = 1
                        avoid_distance = 0
						
                      if data['running_mode'] == 'intelligent_avoidance':
                        running_mode = 2
						
                      if data['running_mode'] == 'intelligent_track':
                        running_mode = 3
                       
                      if data['running_mode'] == 'distance': 
                        avoid_distance = data.get('distance', 0)
                      
                      robot.go([0, 0, 0], 0, 1000)
                      robot.set_body_pose([0, 0, 0], 600)
                      robot.set_body_angle([0, 0, 0], 600)						
                      control.send_action_finish("true")

                    if 'speed_mode' in data:    #tool_SpeedMode工具命令
                      if data['speed_mode'] == 'Low_speed':
                        Speed_Mode = 1
						
                      if data['speed_mode'] == 'High_speed':
                        Speed_Mode = 2
							
                      control.send_action_finish("true")

                    if 'lr' in data:          	#tool_led工具命令
                      rgb_left[0] = data.get('lr', 0)
                      rgb_left[1] = data.get('lg', 0)
                      rgb_left[2] = data.get('lb', 0)
                      rgb_right[0] = data.get('rr', 0)
                      rgb_right[1] = data.get('rg', 0)
                      rgb_right[2] = data.get('rb', 0)
						
                      #限位操作，保证不出现0-255以外的值
                      rgb_left[0] = 255 if(rgb_left[0] > 255) else (0 if(rgb_left[0] < 0) else rgb_left[0])
                      rgb_left[1] = 255 if(rgb_left[1] > 255) else (0 if(rgb_left[1] < 0) else rgb_left[1])
                      rgb_left[2] = 255 if(rgb_left[2] > 255) else (0 if(rgb_left[2] < 0) else rgb_left[2])
                      rgb_right[0] = 255 if(rgb_right[0] > 255) else (0 if(rgb_right[0] < 0) else rgb_right[0])
                      rgb_right[1] = 255 if(rgb_right[1] > 255) else (0 if(rgb_right[1] < 0) else rgb_right[1])
                      rgb_right[2] = 255 if(rgb_right[2] > 255) else (0 if(rgb_right[2] < 0) else rgb_right[2])						
						
                      i2csonar.setRGB(1,rgb_left[0],rgb_left[1],rgb_left[2])
                      i2csonar.setRGB(2,rgb_right[0],rgb_right[1],rgb_right[2])
                      #也可用以下语句替代
                      #i2csonar.setRGB(0,rgb_left[0],rgb_left[1],rgb_left[2])
                      control.send_action_finish("true")

                    if 'count' in data:     	#tool_buzzer工具命令
                      buzzer_count = data.get('count', 0)
                      for num in range(buzzer_count):
                        beep.playTone(3000,200,True)
                      control.send_action_finish("true")

                    if 'actionNum' in data:     #tool_ActionGroup工具命令
                      ActionNum = data.get('actionNum', 0)
                      ExecuteActionNum = data.get('executeNum', 0)
                      
                      if(ActionNum == 3): #唤醒动作
                        for num in ExecuteActionNum:
                          _wake_up()
                        
                      elif(ActionNum == 4): #唤醒奔跑动作
                        for num in ExecuteActionNum:
                          wake_up()
                        
                      elif(ActionNum == 5): #撒娇动作
                        for num in ExecuteActionNum:
                          acting_cute()
                        
                      elif(6<= ActionNum <= 14):
                        robot.action_run(str(ActionNum),ExecuteActionNum)
                        time.sleep(0.1)   #等待0.1s，给其他的线程预留处理动作组运行指令并设置标志位的响应时间
                        #robot.__action_run在动作组执行时置1，其他时间置0
                        while (robot.__action_run != 0):
                          time.sleep(0.001)
                      control.send_action_finish("true")

                except Exception as e:
                    print(f"Action execution error: {e}")
            else:
                pass
				
        else:
            pass
       	
        if(running_mode == 2): 
          avoid_distance = 200 if(avoid_distance <200) else avoid_distance #限位
          avoid(avoid_distance)
          
        elif(running_mode == 3):
          avoid_distance = 200 if(avoid_distance <200) else avoid_distance #限位
          follow(avoid_distance)
			
        #由于接收大模型模块的任务每0.02s执行一次，因此此处主循环延时保持一致，确保指令处理速度与指令接收速度同步
        time.sleep(0.02)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    control.stop()
    time.sleep(0.5)






