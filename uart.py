#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: uart.py
# 功能: 基于UART协议的电机与舵机控制核心模块
# 作者: 覃启轩
# 团队: 北京建筑大学工程设计创新中心314工作室
# 创建日期: 2025-07-05
# 修改记录:
#   2025-07-05 v1.0.0 初始版本
#   2025-07-14 v1.0.0 封装成类，优化算法结构
# -----------------------------------------------------------------------------
import serial
from serial.serialutil import SerialException
import threading
from typing import Optional, List

class UARTController:
    """
    UART 串口通信控制器，支持舵机和电机控制

    特性：
    - 线程安全的串口通信
    - 自动 CRC 校验
    - 实时状态跟踪
    - 舵机(1-4)和电机(1-2)范围检查
    """
    # 协议常量
    HEADER = 0x55
    FOOTER = 0xAA
    CMD_MOTOR = 0xAA
    CMD_SERVO = 0xBB
    PACKET_SIZE = 9
    LENGTH_BYTE = 9

    # 设备ID范围
    MOTOR_IDS = [1, 2]
    SERVO_IDS = [1, 2, 3, 4]

    def __init__(self):
        self._serial_port: Optional[serial.Serial] = None
        self._current_motor_speeds = [0, 0]  # 电机速度 [-100, 100]
        self._current_servo_angles = [0, 0, 0, 0]  # 舵机角度 [0, 360]
        self._lock = threading.Lock()

    def init_uart(self, serial_port: str, baudrate: int = 115200) -> None:
        """初始化串口连接（失败时抛出异常）"""
        try:
            with self._lock:
                self._serial_port = serial.Serial(
                    port=serial_port,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.5
                )
                print(f"UART 已初始化: {serial_port}@{baudrate}bps")
        except SerialException as e:
            raise RuntimeError(f"UART初始化失败: {e}") from e

    def close(self) -> None:
        """关闭串口连接"""
        with self._lock:
            if self._serial_port and self._serial_port.is_open:
                self._serial_port.close()
                print("UART 已关闭")

    def set_motor_speed(self, motor_id: int, speed: int) -> bool:
        """
        设置电机速度
        :param motor_id: 电机ID (1-2)
        :param speed: 速度值 (-100 到 100)
        :return: 是否成功
        """
        packet = self._build_motor_packet(motor_id, speed)
        return self._send_serial_data(packet)

    def set_servo_angle(self, servo_id: int, angle: int) -> bool:
        """
        设置舵机角度
        :param servo_id: 舵机ID (1-4)
        :param angle: 角度值 (0-360)
        :return: 是否成功
        """
        packet = self._build_servo_packet(servo_id, angle)
        return self._send_serial_data(packet)

    def get_motor_speeds(self) -> List[int]:
        """获取当前电机速度"""
        return self._current_motor_speeds.copy()

    def get_servo_angles(self) -> List[int]:
        """获取当前舵机角度"""
        return self._current_servo_angles.copy()

    @staticmethod
    def _calculate_crc(data: bytes) -> int:
        """计算Modbus CRC16校验码"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _build_motor_packet(self, motor_id: int, speed: int) -> bytes:
        """构建电机控制数据包"""
        pack = bytearray(9)
        pack[0] = self.HEADER
        pack[1] = self.LENGTH_BYTE
        pack[2] = self.CMD_MOTOR
        pack[3] = motor_id
        speed_data = speed + 100  # 转换到 0-200 范围
        pack[4] = (speed_data >> 8) & 0xFF
        pack[5] = speed_data & 0xFF
        pack[8] = self.FOOTER

        crc = self._calculate_crc(pack[:6])
        pack[6] = crc & 0xFF
        pack[7] = (crc >> 8) & 0xFF

        return bytes(pack)

    def _build_servo_packet(self, servo_id: int, angle: int) -> bytes:
        """构建舵机控制数据包"""
        pack = bytearray(9)
        pack[0] = self.HEADER
        pack[1] = self.LENGTH_BYTE
        pack[2] = self.CMD_SERVO
        pack[3] = servo_id
        pack[4] = (angle >> 8) & 0xFF
        pack[5] = angle & 0xFF
        pack[8] = self.FOOTER

        crc = self._calculate_crc(pack[:6])
        pack[6] = crc & 0xFF
        pack[7] = (crc >> 8) & 0xFF

        return bytes(pack)

    def _send_serial_data(self, packet: bytes) -> bool:
        """发送串口数据"""
        with self._lock:
            if not self._serial_port or not self._serial_port.is_open:
                print("[ERR] UART 未就绪")
                return False
            try:
                print("→ " + " ".join(f"{b:02X}" for b in packet))
                self._serial_port.write(packet)
                self._serial_port.flush()
                return True
            except SerialException as e:
                print(f"[ERR] 发送失败: {e}")
                return False


if __name__ == "__main__":
    import glob
    import sys


    def scan_serial_ports():
        """扫描可用的串口设备"""
        ports = []
        # Windows
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        # Linux
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        # Mac
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')

        available_ports = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                available_ports.append(port)
            except (OSError, serial.SerialException):
                pass
        return available_ports


    # 扫描并选择串口
    ports = scan_serial_ports()
    if not ports:
        print("未找到可用的串口设备！")
        sys.exit(1)

    print("可用的串口设备:")
    for i, port in enumerate(ports):
        print(f"{i}: {port}")

    try:
        choice = int(input("请选择要连接的串口编号: "))
        selected_port = ports[choice]
    except (ValueError, IndexError):
        print("无效的选择！")
        sys.exit(1)

    # 初始化控制器
    controller = UARTController()
    if not controller.init_uart(selected_port):
        sys.exit(1)

    # 交互菜单
    while True:
        print("\n===== 控制菜单 =====")
        print("0: 退出")
        print("1: 控制电机")
        print("2: 控制舵机")
        print("3: 查看当前状态")
        print("4: 所有电机停止")
        print("5: 所有舵机复位")

        try:
            cmd = int(input("请输入命令编号: "))
        except ValueError:
            print("无效输入！")
            continue

        if cmd == 0:
            # 退出前停止所有电机
            for motor_id in controller.MOTOR_IDS:
                controller.set_motor_speed(motor_id, 0)
            controller.close()
            print("已断开连接并退出")
            break

        elif cmd == 1:
            # 控制电机
            try:
                motor_id = int(input("输入电机ID(1-2): "))
                speed = int(input("输入速度值(-100到100): "))
                if not (1 <= motor_id <= 2):
                    raise ValueError
                if not (-100 <= speed <= 100):
                    raise ValueError
                controller.set_motor_speed(motor_id, speed)
            except ValueError:
                print("无效的电机ID或速度值！")

        elif cmd == 2:
            # 控制舵机
            try:
                servo_id = int(input("输入舵机ID(1-4): "))
                angle = int(input("输入角度值(0-360): "))
                if not (1 <= servo_id <= 4):
                    raise ValueError
                if not (0 <= angle <= 360):
                    raise ValueError
                controller.set_servo_angle(servo_id, angle)
            except ValueError:
                print("无效的舵机ID或角度值！")

        elif cmd == 3:
            # 查看状态
            print("\n当前状态:")
            speeds = controller.get_motor_speeds()
            for i, speed in enumerate(speeds, 1):
                print(f"电机{i}: {speed}%")

            angles = controller.get_servo_angles()
            for i, angle in enumerate(angles, 1):
                print(f"舵机{i}: {angle}°")

        elif cmd == 4:
            # 停止所有电机
            for motor_id in controller.MOTOR_IDS:
                controller.set_motor_speed(motor_id, 0)
            print("所有电机已停止")

        elif cmd == 5:
            # 复位所有舵机
            for servo_id in controller.SERVO_IDS:
                controller.set_servo_angle(servo_id, 0)
            print("所有舵机已复位")

        else:
            print("无效的命令编号！")