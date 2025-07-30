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
#   2025-07-30 v1.0.0 完成approach_ball，approach_area
# -----------------------------------------------------------------------------
from uart import UARTController
import signal
import time
import sys
import atexit
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
        self._serial_port = None  # 显式初始化

        # 注册安全钩子
        self.flag_cleanup = False
        signal.signal(signal.SIGINT, self._emergency_stop)
        signal.signal(signal.SIGTERM, self._emergency_stop)
        atexit.register(self._safe_shutdown)

        try:
            self._init_uart(self.port, self.baudrate)
            self._initialize_state()
        except Exception as e:
            self._safe_shutdown()  # 清理可能残留的状态
            raise RuntimeError(f"初始化失败: {e}") from e

    def _initialize_state(self) -> None:
        """初始化硬件状态"""
        self.stop()  # 停止所有电机
        self.release()  # 复位舵机
        time.sleep(0.3)  # 确保硬件响应

    def _safe_shutdown(self) -> None:
        """线程安全的关闭方法"""
        if self.flag_cleanup:
            return
        self.flag_cleanup=True
        print("\n执行安全复位协议...")
        try:
            # 1. 停止电机（不依赖串口状态）
            if hasattr(self, '_motor_speeds'):
                self.stop()
                time.sleep(0.1)

            # 2. 复位舵机（不依赖串口状态）
            if hasattr(self, '_servo_angles'):
                self.release()
                time.sleep(0.1)

            # 3. 关闭串口（严格检查）
            if self._serial_port is not None:
                if hasattr(self._serial_port, 'is_open'):
                    self._serial_port.close()
                self._serial_port = None
        except Exception as e:
            print(f"[WARN] 安全关闭时出错: {str(e)}")
        finally:
            print("硬件复位完成")

    def _emergency_stop(self, signum=None, frame=None) -> None:
        """信号中断处理"""
        print(f"\n捕获到中断信号 {signum}，紧急停止...")
        self._safe_shutdown()
        sys.exit(1)

    def __del__(self):
        """对象销毁时自动复位"""
        self._safe_shutdown()

    def close(self) -> None:
        super().close()
        self._safe_shutdown()

    def forward(self, speed: int = 50) -> None:
        """前进"""
        self.set_motor_speed(1, -speed)
        self.set_motor_speed(2, -speed)
        self.execute()

    def backward(self, speed: int = 50) -> None:
        """后退"""
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)
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

    def approach_ball(self, x, y) -> None:
        """根据球体位置调整运动方向"""
        if 275 < y <= 640:
            #self.stop
            if x < 210:
                print(f"球体位于下部左侧区域: 位置({x:.1f}, {y:.1f})")
                self.left(5)
            elif x > 382:
                print(f"球体位于下部右侧区域: 位置({x:.1f}, {y:.1f})")
                self.right(5)
            else:
                print(f"球体位于下部中间区域: 位置({x:.1f}, {y:.1f})")
                self.forward(5)
        elif 160 < y <= 300:
            #self.stop
            if x < 210:
                print(f"球体位于中部左侧区域: 位置({x:.1f}, {y:.1f})")
                self.left(6)
            elif x > 382:
                print(f"球体位于中部右侧区域: 位置({x:.1f}, {y:.1f})")
                self.right(6)
            else:
                print(f"球体位于中部中间区域: 位置({x:.1f}, {y:.1f})")
                self.forward(6)
        elif y <= 160:
            #time.sleep(0.3)
            if x < 140:
                print(f"球体位于上部左侧区域: 位置({x:.1f}, {y:.1f})")
                self.left(8)
            elif x > 500:
                print(f"球体位于上部右侧区域: 位置({x:.1f}, {y:.1f})")
                self.right(8)
            else:
                print(f"球体位于上部中间区域: 位置({x:.1f}, {y:.1f})")
                self.forward(8)

    def approach_area(self, x, y) -> None:
        """根据目标区域位置调整运动方向"""
        if 480 < y <= 640:
            print(f"目标位于上部区域: 位置({x:.1f}, {y:.1f})")
            if x < 110:
                self.left(8)
            elif x > 530:
                self.right(8)
            else:
                self.forward(8)
        elif 160 < y <= 480:
            print(f"目标位于中部区域: 位置({x:.1f}, {y:.1f})")
            if x < 140:
                self.left(8)
            elif x > 500:
                self.right(8)
            else:
                self.forward(8)
        elif y <= 160:
            print(f"目标位于下部区域: 位置({x:.1f}, {y:.1f})")
            if x < 160:
                self.left(8)
            elif x > 480:
                self.right(8)
            else:
                self.forward(8)

    def search_ball(self, speed) -> None:
        """搜索球体"""
        self.left(speed)


    def search_area(self, speed) -> None:
        """搜索区域"""
        self.right(speed)



# ===================== 测试程序 =====================

# 电机测试
def main1():
    print("开始测试电机")
    controller = Controller()
    time.sleep(1)

    print("前进")
    controller.forward(50)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    print("后退")
    controller.backward(50)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    print("左转")
    controller.left(50)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    print("右转")
    controller.right(50)
    time.sleep(3)
    controller.stop()
    time.sleep(1)

    controller.close()
    print("电机测试完毕\n")

# 舵机测试
def main2():
    print("开始测试舵机")
    controller = Controller()
    time.sleep(1)

    print("爪子闭合")
    controller.catch()
    time.sleep(1)
    print("爪子张开")
    controller.release()
    time.sleep(1)
    print("爪子闭合")
    controller.catch()
    time.sleep(1)
    print("爪子张开")
    controller.release()
    time.sleep(1)

    controller.close()
    print("舵机测试完毕\n")

if __name__ == '__main__':
    main1()
    main2()