import time
import sys
import math
sys.path.append('../lib')
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB2')
cmd = MotorCmd()
data = MotorData()

# 获取减速比
gear_ratio = queryGearRatio(MotorType.GO_M8010_6)
print(f"减速比: {gear_ratio}")

# 正弦波参数
min_angle = 0.0       # 最小角度
max_angle = 180.0      # 最大角度
frequency = 0.5       # 频率：0.5Hz
start_time = None     # 开始时间
initial_position = None  # 初始位置
target_initialized = False  # 目标初始化标志

try:
    while True:  
        # 配置电机参数
        data.motorType = MotorType.GO_M8010_6
        cmd.motorType = MotorType.GO_M8010_6
        
        # 使用FOC闭环控制模式实现角度控制
        cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        cmd.id = 0
        
        # 获取当前电机位置
        serial.sendRecv(cmd, data)
        current_position_rad = data.q / gear_ratio if gear_ratio != 0 else 0
        current_position_deg_motor = current_position_rad * 180.0 / 3.1415926
        current_position_deg = (360 - current_position_deg_motor) % 360
        if current_position_deg > 180:
            current_position_deg -= 360
            
        # 初始化：记录初始位置和开始时间
        if start_time is None:
            start_time = time.time()
            initial_position = current_position_deg
            print(f"初始位置: {initial_position:.1f}度")
            
        # 计算目标角度
        if start_time is not None:
            current_time = time.time() - start_time
            
            # 计算正弦波目标角度
            center = (min_angle + max_angle) / 2  # 中心点: 25度
            amplitude = (max_angle - min_angle) / 2  # 振幅: 25度
            sine_target = center + amplitude * math.sin(2 * math.pi * frequency * current_time)
            
            # 如果是第一次初始化目标，进行平滑过渡
            if not target_initialized:
                # 计算当前位置与目标起点的差值
                position_diff = sine_target - initial_position
                
                # 如果差值较大，进行平滑过渡
                if abs(position_diff) > 10:  # 差值超过10度时进行平滑处理
                    # 使用线性插值缓慢接近目标
                    transition_factor = min(1.0, current_time / 2.0)  # 2秒内完成过渡
                    target_position_deg = initial_position + position_diff * transition_factor
                    if transition_factor >= 1.0:
                        target_initialized = True
                else:
                    target_position_deg = sine_target
                    target_initialized = True
            else:
                target_position_deg = sine_target
        
        # 将刻度盘角度转换为电机坐标系角度
        # 刻度盘: 顺时针为正  电机坐标系: 逆时针为正
        motor_target_deg = (360 - target_position_deg) % 360
        target_position_rad = motor_target_deg * 3.14159 / 180.0
        
        # 角度控制设置 - 考虑减速比，控制参数是输入轴参数
        cmd.q = target_position_rad * gear_ratio    # 输入轴目标位置 (rad)
        cmd.dq = 0.0                               # 目标速度 (rad/s)
        cmd.kp = 0.5                             # 位置增益（增强响应）
        cmd.kd = 0.01                               # 速度增益
        cmd.tau = 0.0                             # 附加扭矩 (Nm)
        
        # 发送指令并接收反馈数据
        serial.sendRecv(cmd, data)
        
        # 解析电机反馈数据
        output_position_rad = data.q / gear_ratio if gear_ratio != 0 else 0
        output_position_deg_motor = output_position_rad * 180.0 / 3.1415926
        
        # 转换回刻度盘坐标系，并归一化到 [-180, 180]
        output_position_deg = (360 - output_position_deg_motor) % 360
        if output_position_deg > 180:
            output_position_deg -= 360
        
        # 速度方向转换
        output_velocity_dps_motor = (data.dq / gear_ratio if gear_ratio != 0 else 0) * 180.0 / 3.14159
        output_velocity_dps = -output_velocity_dps_motor
        
        # 打印状态信息
        print(f"\n--- 电机状态 ---")
        print(f"目标角度(刻度盘): {target_position_deg:.1f}")
        print(f"当前位置(电机转子): {output_position_deg:.1f}")
        if abs(output_position_deg) < 0.1:
            print(" 提示：当前位置等价于 0")
        print(f"当前速度(电机转子): {output_velocity_dps:.1f}/s")
        print(f"电机温度: {data.temp} C")
        print(f"错误代码: {data.merror}")
        
        time.sleep(0.01)  # 10ms 控制周期
            
except KeyboardInterrupt:
    # 安全停止
    cmd.q = 0
    cmd.dq = 0
    cmd.kp = 0
    cmd.kd = 0
    cmd.tau = 0
    serial.sendRecv(cmd, data)
    print("\n电机已安全停止")

