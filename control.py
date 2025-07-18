#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: control.py
# 功能: 运动控制核心模块
# 团队: 北京建筑大学工程设计创新中心314工作室
# 创建日期: 2025-07-05
# 修改记录:
#   2025-07-05 v1.0.0 初始版本 樊彧创建并完成基本架构编写
#   2025-07-18 v1.0.0 初始版本 重构，增加缓冲区 增加approch
# -----------------------------------------------------------------------------
from uart import UARTController
import time
from config import CATCH_ANGLE,RELEASE_ANGLE,UART_PORT,DEFAULT_RESOLUTION
frame_width, frame_height = DEFAULT_RESOLUTION

class Controller(UARTController):
    def __init__(self, port: str = UART_PORT, baudrate: int = 115200):
        """
        运动控制器 (依赖底层UARTController的范围检查)

        参数:
            port: 串口设备路径 (默认从config导入)
            baudrate: 波特率 (默认115200)
        """
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.init_uart(self.port,self.baudrate)

        # 初始化状态
        self.stop()  # 停止所有电机
        self.release()  # 复位舵机
        time.sleep(0.1)  # 确保硬件响应

    def forward(self, speed: int = 50) -> None:
        """前进"""
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)
        self.execute()

    def backward(self, speed: int = 50) -> None:
        """后退"""
        self.set_motor_speed(1, -speed)
        self.set_motor_speed(2, -speed)
        self.execute()

    def left(self, speed: int = 30) -> None:
        """左转"""
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, -speed)
        self.execute()

    def right(self, speed: int = 30) -> None:
        """右转"""
        self.set_motor_speed(1, -speed)
        self.set_motor_speed(2, speed)
        self.execute()

    def stop(self) -> None:
        """停止所有电机"""
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)
        self.execute()

    # 舵机爪子张开和闭合的角度去config.py里调整，不要在这里调整
    def catch(self, angle1 = CATCH_ANGLE[0], angle2 = CATCH_ANGLE[1]) -> None:
        """执行抓取动作"""
        self.set_servo_angle(1, angle1)
        self.set_servo_angle(2, angle2)
        self.execute()
        time.sleep(0.3)  # 等待动作完成

    # 舵机爪子张开和闭合的角度去config.py里调整，不要在这里调整
    def release(self, angle1 = RELEASE_ANGLE[0], angle2 = RELEASE_ANGLE[1]) -> None:
        """执行释放动作"""
        self.set_servo_angle(1, angle1)
        self.set_servo_angle(2, angle2)
        self.execute()
        time.sleep(0.3)  # 等待动作完成

    def approach_ball(self, x: float, y: float) -> None:
        """
        根据球的位置控制机器人接近球体
        
        参数:
            x: 球在图像中的x坐标 (0到frame_width)
            y: 球在图像中的y坐标 (0到frame_height)
            frame_width: 图像宽度
            frame_height: 图像高度
        """
        # 计算与图像中心的偏差 (比例控制)
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        # x方向偏差 (正数表示球在右侧，负数表示球在左侧)
        x_error = x - center_x
        # y方向偏差 (正数表示球在下方，负数表示球在上方)
        y_error = y - center_y
        
        # 归一化偏差 (范围 -1.0 到 1.0)
        x_error_norm = x_error / center_x
        y_error_norm = y_error / center_y
        
        # 基础速度 (接近速度)
        base_speed = 20
        
        # 计算左右轮速度 (差速控制)
        # 当球在正前方时，左右轮速度相等
        # 当球在左侧时，左轮速度减小，右轮速度增大
        # 当球在右侧时，右轮速度减小，左轮速度增大
        left_speed = base_speed * (1 + x_error_norm)
        right_speed = base_speed * (1 - x_error_norm)
        
        # 限制速度范围 (-100 到 100)
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # 根据y方向误差调整总体速度
        # 当球在远处时，速度较快
        # 当球在近处时，速度较慢
        distance_factor = 1.0 - min(1.0, max(0.0, y_error_norm))
        speed_factor = 0.5 + 0.5 * distance_factor
        
        left_speed = int(left_speed * speed_factor)
        right_speed = int(right_speed * speed_factor)
        
        # 设置电机速度
        self.set_motor_speed(1, left_speed)
        self.set_motor_speed(2, right_speed)
        self.execute()
        
        # 打印调试信息
        print(f"接近球体: 位置({x:.1f}, {y:.1f}), "
              f"速度(L:{left_speed}, R:{right_speed})")

    def approach_area(self, x: float, y: float) -> None:
        """
        根据区域位置控制机器人接近安全区域
        
        参数:
            x: 区域在图像中的x坐标 (0到frame_width)
            y: 区域在图像中的y坐标 (0到frame_height)
            frame_width: 图像宽度
            frame_height: 图像高度
        """
        # 计算与图像中心的偏差 (比例控制)
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        # x方向偏差 (正数表示区域在右侧，负数表示区域在左侧)
        x_error = x - center_x
        # y方向偏差 (正数表示区域在下方，负数表示区域在上方)
        y_error = y - center_y
        
        # 归一化偏差 (范围 -1.0 到 1.0)
        x_error_norm = x_error / center_x
        y_error_norm = y_error / center_y
        
        # 基础速度 (接近速度)
        base_speed = 20
        
        # 计算左右轮速度 (差速控制)
        left_speed = base_speed * (1 + x_error_norm)
        right_speed = base_speed * (1 - x_error_norm)
        
        # 限制速度范围 (-100 到 100)
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))
        
        # 根据y方向误差调整总体速度
        # 当区域在远处时，速度较快
        # 当区域在近处时，速度较慢
        distance_factor = 1.0 - min(1.0, max(0.0, y_error_norm))
        speed_factor = 0.4 + 0.6 * distance_factor  # 接近区域时更慢
        
        left_speed = int(left_speed * speed_factor)
        right_speed = int(right_speed * speed_factor)
        
        # 设置电机速度
        self.set_motor_speed(1, left_speed)
        self.set_motor_speed(2, right_speed)
        self.execute()
        
        # 打印调试信息
        print(f"接近区域: 位置({x:.1f}, {y:.1f}), "
              f"速度(L:{left_speed}, R:{right_speed})")

    def search_ball(self, speed = 30) -> None:
        """搜索球体"""
        self.left(speed)
        self.stop()

    def search_area(self, speed = 30) -> None:
        """搜索区域"""
        self.right(speed)
        self.stop()


# ===================== 测试程序 =====================

# 电机测试
def main1():
    controller = Controller()
    time.sleep(1)

    controller.forward(50)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    controller.backward(50)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    controller.left(50)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    controller.right(50)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    controller.close()

# 舵机测试
def main2():
    controller = Controller()
    time.sleep(1)

    controller.catch()
    time.sleep(1)
    controller.release()
    time.sleep(1)
    controller.catch()
    time.sleep(1)
    controller.release()
    time.sleep(1)

    controller.close()

if __name__ == '__main__':
    main1()
    main2()
