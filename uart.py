#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: uart.py
# 功能: 基于UART协议的电机与舵机控制核心模块
# 作者: 覃启轩
# 团队: 北京建筑大学314工作室
# 创建日期: 2025-07-05
# 修改记录:
#   2025-07-05 v1.0.0 初始版本
# -----------------------------------------------------------------------------
import serial
from serial.serialutil import SerialException
import time
import threading
from typing import Optional

# 全局设备对象
_serial_port: Optional[serial.Serial] = None

# 设备状态记录
_current_motor_speeds = [0, 0]  # 电机1和2的速度值 [-100, 100]
_current_servo_angles = [0, 0, 0, 0]  # 舵机1-4的角度值 [0, 360]

# 根据下位机代码定义的常量
HEADER = 0x55  # 包头
FOOTER = 0xAA  # 包尾
CMD_MOTOR = 0xAA  # 电机控制命令
CMD_SERVO = 0xBB  # 舵机控制命令
PACKET_SIZE = 9  # 数据包固定大小
LENGTH_BYTE = 9  # 下位机包长

# 设备ID定义
MOTOR_IDS = [1, 2]  # 电机ID列表
SERVO_IDS = [1, 2, 3, 4]  # 舵机ID列表


def init_uart(serial_port: str, baudrate: int = 115200) -> None:
    """初始化 UART 并启动接收线程"""
    global _serial_port
    try:
        _serial_port = serial.Serial(
            port=serial_port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5
        )
        print(f"UART 已初始化: {serial_port}@{baudrate}bps")
        threading.Thread(target=receive_thread, daemon=True).start()
    except SerialException as e:
        print(f"UART 初始化失败: {e}")
        _serial_port = None


def calculate_crc16_modbus(data: bytes) -> int:
    """
    CRC-16/Modbus 计算（与C实现完全一致）
    参数:
        data: 输入数据（字节形式）
    返回:
        CRC-16 值（整数，范围 0x0000-0xFFFF）
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_motor_pack(motor_id: int, speed: int) -> bytes:
    """
    参数:
        motor_id: 电机ID（1或2）
        speed: 速度值（-100到100）
    返回:
        完整的数据包（bytes类型）
    """
    # 初始化数据包（9字节）
    pack = bytearray(9)

    # 固定包头和包尾
    pack[0] = HEADER  # 包头
    pack[1] = LENGTH_BYTE  # 包长（现在设为9）
    pack[2] = CMD_MOTOR  # 电机命令字
    pack[8] = FOOTER  # 包尾

    # 设置电机ID
    if motor_id == 1:
        pack[3] = 0x01
    elif motor_id == 2:
        pack[3] = 0x02
    else:
        raise ValueError("电机ID必须是1或2")

    # 处理速度值（范围-100到100）
    speed_data = speed + 100  # 转换为0-200范围
    if not 0 <= speed_data <= 200:
        raise ValueError("速度值必须在-100到100之间")

    # 将速度值写入第4、5字节（小端序）
    pack[4] = (speed_data >> 8) & 0xFF  # 高字节
    pack[5] = speed_data & 0xFF  # 低字节

    # 计算CRC（前6字节：索引0-5）
    crc = calculate_crc16_modbus(bytes(pack[:6]))

    # 将CRC写入第6、7字节（低字节在前！）
    pack[6] = crc & 0xFF  # CRC低字节
    pack[7] = (crc >> 8) & 0xFF  # CRC高字节

    return bytes(pack)


def build_servo_pack(servo_id: int, angle: int) -> bytes:
    """
    参数:
        servo_id: 舵机ID（1~4）
        angle: 角度值（0-360）
    返回:
        完整的数据包（bytes类型）
    """
    # 初始化数据包（9字节）
    pack = bytearray(9)

    # 固定包头和包尾
    pack[0] = HEADER  # 包头
    pack[1] = LENGTH_BYTE  # 包长（现在设为9）
    pack[2] = CMD_SERVO  # 舵机命令字
    pack[8] = FOOTER  # 包尾

    # 设置舵机ID（1~4）
    if 1 <= servo_id <= 4:
        pack[3] = servo_id
    else:
        raise ValueError("舵机ID必须是1~4")

    # 检查角度范围
    if not 0 <= angle <= 360:
        raise ValueError("角度值必须在0-360之间")

    # 将角度值写入第4、5字节（高字节在前）
    pack[4] = (angle >> 8) & 0xFF  # 高字节
    pack[5] = angle & 0xFF  # 低字节

    # 计算CRC（前6字节：索引0-5）
    crc = calculate_crc16_modbus(bytes(pack[:6]))

    # 将CRC写入第6、7字节（低字节在前）
    pack[6] = crc & 0xFF  # CRC低字节
    pack[7] = (crc >> 8) & 0xFF  # CRC高字节

    return bytes(pack)


def parse_packet(packet: bytes) -> None:
    """
    解析下位机回显的包，并更新 _current_* 状态
    """
    if len(packet) != PACKET_SIZE:
        print(f"[ERR] 包长度不对: {len(packet)} (期望 {PACKET_SIZE})")
        return
    if packet[0] != HEADER or packet[-1] != FOOTER:
        print(f"[ERR] 包头尾错误: {packet[0]:02X}/{packet[-1]:02X}")
        return

    length_byte = packet[1]
    if length_byte != LENGTH_BYTE:
        print(f"[ERR] length_byte 不对: {length_byte} (期望 {LENGTH_BYTE})")
        return

    # 计算所有参与CRC的字节（0-5）
    crc_data = packet[0:6]
    calc_crc = calculate_crc16_modbus(crc_data)
    # 正确接收CRC值（小端序）
    recv_crc = packet[6] | (packet[7] << 8)

    if calc_crc != recv_crc:
        print(f"[ERR] CRC 验证失败: 接收={recv_crc:04X}, 计算={calc_crc:04X}")
        return

    cmd, dev, h, l = packet[2], packet[3], packet[4], packet[5]
    raw = (h << 8) | l

    if cmd == CMD_MOTOR:
        if dev in MOTOR_IDS:
            speed = raw - 100
            _current_motor_speeds[dev - 1] = speed
            print(f"[OK] 电机{dev} → 速度 {speed}%")
        else:
            print(f"[ERR] 未知电机ID {dev}")
    elif cmd == CMD_SERVO:
        if dev in SERVO_IDS:
            _current_servo_angles[dev - 1] = raw
            print(f"[OK] 舵机{dev} → 角度 {raw}°")
        else:
            print(f"[ERR] 未知舵机ID {dev}")
    else:
        print(f"[ERR] 未知 cmd_type {cmd:02X}")


def send_serial_data(packet: bytes) -> bool:
    """发送一帧到下位机"""
    if not _serial_port or not _serial_port.is_open:
        print("[ERR] UART 未就绪")
        return False
    try:
        print("→ " + " ".join(f"{b:02X}" for b in packet))
        _serial_port.write(packet)
        _serial_port.flush()
        return True
    except SerialException as e:
        print(f"[ERR] 发送失败: {e}")
        return False


def set_motor_speed(motor_id: int, speed: int) -> None:
    if motor_id not in MOTOR_IDS or not -100 <= speed <= 100:
        print(f"[ERR] 参数错误: motor_id={motor_id}, speed={speed}")
        return
    pkt = build_motor_pack(motor_id, speed)
    if send_serial_data(pkt):
        print(f"Set motor{motor_id} → {speed}%")


def set_servo_angle(servo_id: int, angle: int) -> None:
    if servo_id not in SERVO_IDS or not 0 <= angle <= 360:
        print(f"[ERR] 参数错误: servo_id={servo_id}, angle={angle}")
        return
    pkt = build_servo_pack(servo_id, angle)
    if send_serial_data(pkt):
        print(f"Set servo{servo_id} → {angle}°")


def receive_thread() -> None:
    """持续读取并按 9 字节包格式解析"""
    if not _serial_port:
        return
    print("UART 接收线程启动")
    buf = bytearray()
    while _serial_port.is_open:
        data = _serial_port.read_all()
        if data:
            buf.extend(data)
            # 尝试拆包
            while len(buf) >= PACKET_SIZE:
                # 找到合法头尾
                idx = next((i for i in range(len(buf) - 8)
                            if buf[i] == HEADER and buf[i + 8] == FOOTER), None)
                if idx is None:
                    # 没有找到完整包，保留最后8字节
                    if len(buf) > 8:
                        buf = buf[-8:]
                    break
                frame = bytes(buf[idx:idx + PACKET_SIZE])
                del buf[:idx + PACKET_SIZE]
                parse_packet(frame)
        else:
            time.sleep(0.01)


def close_uart() -> None:
    global _serial_port
    if _serial_port and _serial_port.is_open:
        _serial_port.close()
        print("UART 已关闭")


def get_motor_speeds() -> list:
    return _current_motor_speeds.copy()


def get_servo_angles() -> list:
    return _current_servo_angles.copy()


if __name__ == "__main__":
    init_uart("COM3", 115200)
    try:
        set_motor_speed(1, 75)
        time.sleep(1)
        set_motor_speed(2, -50)
        time.sleep(1)
        set_servo_angle(1, 90)
        time.sleep(1)
        set_servo_angle(2, 180)
        time.sleep(1)
        set_motor_speed(1, 0)
        set_motor_speed(2, 0)
        time.sleep(0.5)
        print("当前电机速度:", get_motor_speeds())
        print("当前舵机角度:", get_servo_angles())
    finally:
        close_uart()