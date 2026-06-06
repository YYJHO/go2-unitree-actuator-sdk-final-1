import time
import sys
sys.path.append('../lib')
from unitree_actuator_sdk import *

# 初始化串口和电机控制结构
serial = SerialPort('/dev/ttyUSB2')
cmd = MotorCmd()
data = MotorData()

# 获取电机减速比
gear_ratio = queryGearRatio(MotorType.GO_M8010_6)
print(f"减速比: {gear_ratio}")

# 设置目标转速 (RPM) - 输出轴转速
target_speed_rpm = -1.0

try:
    while True:
        # 配置电机参数
        data.motorType = MotorType.GO_M8010_6
        cmd.motorType = MotorType.GO_M8010_6
        
        # 使用FOC闭环速度控制模式
        cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        cmd.id = 0
        
        # 将输出轴转速(RPM)转换为输入轴转速(rad/s)
        target_speed_rps = target_speed_rpm / 60.0  # 转换为转每秒
        target_speed_rad_per_sec = target_speed_rps * 2 * 3.1415926  # 转换为rad/s
        
        # 计算输入轴目标速度
        cmd.dq = target_speed_rad_per_sec * gear_ratio  # 输入轴目标速度 (rad/s)
        cmd.q = 0.0                                     # 位置设为0（纯速度控制）
        cmd.kp = 0.0                                    # 位置增益设为0
        cmd.kd = 0.2                                    # 速度控制器增益
        cmd.tau = 0.0                                   # 附加扭矩 (Nm)
        
        # 发送指令并接收反馈数据
        serial.sendRecv(cmd, data)
        
        # 输出轴实际速度计算
        input_actual_speed = data.dq  # 输入轴实际速度 (rad/s)
        output_speed_rad_per_sec = input_actual_speed / gear_ratio if gear_ratio != 0 else 0
        output_speed_rps = output_speed_rad_per_sec / (2 * 3.1415926)
        output_speed_rpm = output_speed_rps * 60  # 转换为RPM
        
        # 打印电机状态
        print(f"\n--- 电机状态 ---")
        print(f"目标转速(输出轴): {target_speed_rpm:.1f} RPM")
        print(f"实际转速(输出轴): {output_speed_rpm:.2f} RPM")
        print(f"电机温度: {data.temp} C")
        print(f"错误代码: {data.merror}")
        print(f"减速比: {gear_ratio}")
        
        time.sleep(0.01)  # 10ms控制周期
            
except KeyboardInterrupt:
    # 安全停止
    cmd.q = 0
    cmd.dq = 0
    cmd.kp = 0
    cmd.kd = 0
    cmd.tau = 0
    serial.sendRecv(cmd, data)
    print("\n电机已安全停止")
