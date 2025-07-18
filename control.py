#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: control.py
# 功能: 运动控制核心模块
# 团队: 北京建筑大学工程设计创新中心314工作室
# 创建日期: 2025-07-05
# 修改记录:
#   2025-07-05 v1.0.0 初始版本 樊彧创建并完成基本架构编写
#   2025-07-18 v1.0.0 初始版本 重构，增加缓冲区
# -----------------------------------------------------------------------------
from uart import UARTController
import time
from config import CATCH_ANGLE,RELEASE_ANGLE,UART_PORT

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

    def approach_ball(self,x,y) -> None:
        pass

    def approach_area(self,x,y) -> None:
        pass

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
