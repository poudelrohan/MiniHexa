# MiniHexa

[English](https://github.com/Hiwonder/MiniHexa/blob/main/README.md) | 中文

<p align="center">
  <img src="./sources/images/minihexa.png" alt="MiniHexa Logo" width="600"/>
</p>

## 产品概述

MiniHexa 是由幻尔科技（Hiwonder）开发的教育型六足机器人。它采用紧凑的设计，配备六条腿，能够实现灵活多样的运动。机器人支持多种编程方式，包括 Python（MicroPython）、Arduino、Scratch 和 APP 遥控，非常适合学习机器人技术、编程和 AI 应用。

MiniHexa 配备了 AI 功能，包括计算机视觉、语音交互和大语言模型集成。它为学生和爱好者提供了一个全面的平台，用于探索机器人技术、人工智能和具身智能应用。

## 官方资源

### 幻尔科技官方
- **官方网站**: [https://www.hiwonder.com/](https://www.hiwonder.com/)
- **在线教程**: [https://wiki.hiwonder.com/projects/miniHexa/en/latest/](https://wiki.hiwonder.com/projects/miniHexa/en/latest/)
- **技术支持**: support@hiwonder.com

### 视频演示

- [MiniHexa 产品介绍](https://www.youtube.com/watch?v=BYNRlQRvBf4)
- [六足机器人 miniHexa 超声波避障演示](https://www.youtube.com/shorts/Ahp4gRtIWlY)
- [六足机器人 miniHexa 智能传感器探索](https://www.youtube.com/shorts/JSn-nrMnQPY)
- [六足机器人 miniHexa PC 上位机控制](https://www.youtube.com/shorts/OPfjU2Akzv8)
- [认识 miniHexa - 智能有趣的入门级六足机器人](https://www.youtube.com/shorts/dciMLGKGKoI)

## 主要功能

### 多种编程方式
- **Python 编程** - MicroPython 开发环境
- **Arduino 编程** - 兼容 Arduino IDE 的项目
- **Scratch 编程** - 可视化积木式编程
- **APP 遥控** - 移动应用无线控制
- **上位机控制** - 基于 PC 的控制和动作编辑器

### AI 视觉功能
- **目标检测** - 实时目标检测与识别
- **颜色识别** - 高级颜色检测与识别
- **目标跟踪** - AI 驱动的目标跟踪
- **视觉处理** - 计算机视觉应用

### AI 语音功能
- **语音识别** - 语音识别能力
- **语音交互** - 自然语言语音命令
- **AI 语音项目** - 语音控制应用

### AI 大模型集成
- **多模态 AI** - 大语言模型集成
- **具身智能** - AI 驱动的自主行为
- **离线 AI 模型** - 本地 AI 模型部署

### 运动控制
- **六足步态** - 六足行走模式
- **多方向运动** - 前进、后退、转向
- **动作编辑器** - 自定义动作序列创建
- **串口通信** - 基于 UART 的控制协议

## 硬件配置
- **控制器**: 基于 ESP32 的微控制器
- **舵机**: 6 个高扭矩舵机用于腿部控制
- **传感器**: 摄像头、麦克风、距离传感器
- **通信方式**: WiFi、蓝牙、UART 串口
- **电源**: 可充电电池系统
- **扩展**: GPIO 引脚用于额外传感器和模块

## 项目结构

```
MiniHexa/
├── app-remote-control/              # 移动 APP 遥控
├── arduino-programming-projects/    # Arduino IDE 项目
├── host-control-and-action-editor/ # PC 控制软件和动作编辑器
├── python-programming-projects/     # MicroPython 项目
│   ├── 5.1 MicroPython开发环境搭建
│   ├── 5.2 运动控制基本教程
│   ├── 5.3 二次开发项目课程
│   ├── 5.4 AI视觉项目课程
│   ├── 5.5 AI语音项目课程
│   ├── 5.6 AI大模型应用课程
│   └── 5.7 AI大模型离线课程
├── scratch-programming-projects/    # Scratch 可视化编程
└── serial-communication-tutorial/   # UART 通信指南
```

## 快速开始

### Python 编程
1. 搭建 MicroPython 开发环境
2. 参考 `python-programming-projects/` 中的教程
3. 从基本运动控制开始
4. 探索 AI 视觉和语音项目

### Arduino 编程
1. 安装 Arduino IDE
2. 打开 `arduino-programming-projects/` 中的项目
3. 上传代码到 MiniHexa
4. 测试和修改示例

### Scratch 编程
1. 安装 Scratch 软件
2. 从 `scratch-programming-projects/` 加载项目
3. 使用拖放积木创建可视化程序

### APP 控制
1. 安装 MiniHexa 移动应用
2. 通过 WiFi/蓝牙连接机器人
3. 使用遥控界面

## 版本信息
- **当前版本**: MiniHexa v1.0
- **支持平台**: ESP32、MicroPython、Arduino、Scratch

### 相关技术
- [MicroPython](https://micropython.org/) - 微控制器 Python
- [Arduino](https://www.arduino.cc/) - 开源电子平台
- [Scratch](https://scratch.mit.edu/) - 可视化编程语言
- [ESP32](https://www.espressif.com/en/products/socs/esp32) - 微控制器平台

---

**注意**: 本仓库包含 MiniHexa 六足机器人的教育材料和项目。详细教程和文档请参阅幻尔科技官方文档。
