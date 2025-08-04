# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: main.py
# 功能: 工程实践与创新能力大赛智能救援赛项
# 团队: 北京建筑大学工程设计创新中心314工作室
# 创建日期: 2025-07-04
# 修改记录:
#   2025-07-05 v1.0.0 初始版本 樊彧创建并完成基本架构编写
#   2025-07-05 樊彧 优化了算法结构
#   2025-07-31 补充：
#       1. 当识别到中部中间区域存在大于3个球时执行特殊动作
#       2. state1时检测球是否仍然存在
#       3. state3时针对中部中间区域的特殊处理
#       4. state2时实时检测球体位置
#   2025-08-01 修复：确保返回state0时爪子松开
# -----------------------------------------------------------------------------
"""
状态机说明:
state 0: 执行寻找球
state 1: 已找到球，执行抓球
state 2: 已抓到球，执行寻找area
state 3: 已找到area，执行放球和后续操作
"""

import cv2
from ultralytics import YOLO
import time
import numpy as np
from control import Controller
from vision import VideoStream, VISION
from config import label_balls, label_area, RESCUE_MODEL, DEFAULT_RESOLUTION, HOLDING_AREA, CENTER_REGION, READY_AREA, \
    CATCH_AREA, CROSS_CENTER_REGION

model = YOLO(str(RESCUE_MODEL))
model.to('cuda:0')  # 使用GPU推理

stream = VideoStream(save_output=True)
controller = Controller()
vision = VISION(model, label_balls, label_area)

frame_width, frame_height = DEFAULT_RESOLUTION

# 创建视频窗口
cv2.namedWindow("Rescue Robot Vision", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Rescue Robot Vision", frame_width, frame_height)

state: int = 0
state_list = ["执行寻找球", "已找到球，执行抓球", "已抓到球，执行寻找area", "已找到area，执行放球和后续操作"]

print("准备就绪")

# 初始化参数
start_flag = 0
frame_count = 0
detect_interval = 1  # 每x帧进行一次检测
start_time = None
results = None  # 存储模型检测结果
running = True
last_flag_found_ball = False
last_flag_found_area = False
last_time = time.time()
MAX_STATE3_DURATION = 15  # 状态3最大持续时间
# 追踪球的消失
ball_disappear_counter = 0

# 追踪爪子状态
claw_state = "open"  # "open"或"close"


def is_ball_center_in_area(ball_center, area_box):
    if ball_center is None or area_box is None:
        return False

    x, y = ball_center
    ax1, ay1, ax2, ay2 = area_box

    # 检查球中心是否在安全区内
    return (ax1 <= x <= ax2 and
            ay1 <= y <= ay2)


# 检测球体是否在抓取后位置
def is_ball_in_holding_area(ball_center):
    if ball_center is None:
        return False

    x, y = ball_center
    return (HOLDING_AREA[0] <= x <= HOLDING_AREA[2] and
            HOLDING_AREA[1] <= y <= HOLDING_AREA[3])


while running:
    # 读取帧
    frame = stream.read_frame()
    frame_count += 1

    # 处理帧并显示状态
    display_frame = frame.copy()

    if frame_count % detect_interval == 0:
        results = model(frame, verbose=False)

    # 每0.5s打印一次当前状态
    current_time = time.time()
    if current_time > last_time + 0.5:
        last_time = current_time
        print(f"第{frame_count}次运行。当前状态：{state_list[state]}，爪子状态：{claw_state}")

    if start_flag == 0:
        controller.start_catch()
        start_flag = 1

    # 执行寻找球
    if state == 0:
        # 确保爪子是松开的
        if claw_state != "open":
            controller.release()
            claw_state = "open"
            time.sleep(0.3)  # 等待释放完成

        # 2. 检测目标球
        flag_found_ball, ball_data = vision.detect_closest_ball(results)

        if flag_found_ball:
            if not last_flag_found_ball:
                last_flag_found_ball = True
                controller.stop()
                ball_disappear_counter = 0  # 重置消失计数器

            # 解包ball_data获取坐标数据等
            bcx, bcy = ball_data['center']

            # 在爪子内，执行抓球
            if vision.is_ready_to_catch(bcx, bcy):
                state = 1
            else:
                controller.approach_ball(bcx, bcy)
        else:
            controller.search_ball(speed=1500)  # 没找到球，旋转车体进行找球
            last_flag_found_ball = False

            # 增加消失计数
            ball_disappear_counter += 1

    # 已找到球，执行抓球
    if state == 1:

        flag_found_current, current_ball_data = vision.detect_closest_ball(results)

        # 如果是首次进入状态1，执行抓取动作
        if claw_state == "open":
            controller.catch()
            claw_state = "close"  # 更新爪子状态
            start_time = time.time()  # 记录抓取开始时间
            print("执行抓取动作")

        # 检查抓取是否成功
        elapsed = time.time() - start_time if start_time else 0

        # 增加检测窗口到2秒，并添加多帧验证
        if elapsed >= 0.5:  # 等待0.5秒后再开始检测
            # 检查成功条件：球是否在HOLDING_AREA区域
            flag_found, current_ball_data = vision.detect_closest_ball(results)
            if flag_found:
                ball_in_position = is_ball_in_holding_area(current_ball_data['center'])
            else:
                ball_in_position = False

            if ball_in_position:
                print("抓取成功(球在HOLDING_AREA内)，进入状态2")
                start_time = None
                state = 2
            elif elapsed > 2.0:  # 2秒后仍失败则放弃
                print("抓取失败，返回状态0")
                controller.release()
                claw_state = "open"
                start_time = None
                state = 0

    # 已抓到球，执行寻找area
    if state == 2:

        # 检测球体是否在抓取后位置
        flag_found_ball, ball_data = vision.detect_closest_ball(results)

        # 安全处理：当球体未被检测到时
        if flag_found_ball and ball_data is not None:
            ball_center = ball_data['center']
            ball_in_position = is_ball_in_holding_area(ball_center)
        else:
            ball_in_position = False

        # 寻找安全区
        flag_found_area, area_data = vision.detect_area(results)

        # 寻找区域
        if flag_found_area:
            if not last_flag_found_area:
                last_flag_found_area = True
                controller.stop()

            acx, acy = area_data['center']
            controller.approach_area(acx, acy)

            # 检查安全区中心是否在指定区域
            if (CENTER_REGION[0] <= acx <= CENTER_REGION[2] and
                    CENTER_REGION[1] <= acy <= CENTER_REGION[3]):
                print("安全区位于指定区域，进入状态3")
                state = 3
                state3_start_time = time.time()  # 记录状态3开始时间
                controller.stop()  # 先停止移动
        else:
            controller.search_area(speed=800)
            last_flag_found_area = False

    # 已到达area，执行放球和后续操作
    if state == 3:
        # 超时检查
        if time.time() - state3_start_time > 5:
            print("状态3超时（8秒），强制松爪后退")
            controller.release()
            claw_state = "open"
            controller.backward(1500)
            time.sleep(3)
            controller.stop()
            state = 0
            continue

        # 1. 前冲2秒
        print("执行前冲2秒")
        controller.forward(2000)  # 前进
        time.sleep(2)
        controller.stop()

        # 2. 停止1秒
        print("停止1秒")
        time.sleep(1)

        controller.release()
        claw_state = "open"
        controller.backward(2000)
        time.sleep(2)
        controller.stop()
        state = 0

    # 显示和保存
    # 准备抓取区域
    cv2.rectangle(
        frame,
        (CATCH_AREA[0], CATCH_AREA[1]),
        (CATCH_AREA[2], CATCH_AREA[3]),
        (255, 255, 255),  # 白色边框
        2
    )

    # 绘制中部中间区域
    cv2.rectangle(
        frame,
        (CENTER_REGION[0], CENTER_REGION[1]),
        (CENTER_REGION[2], CENTER_REGION[3]),
        (0, 255, 255),  # 黄色边框
        2
    )

    # 绘制抓取后球体位置检测区域
    cv2.rectangle(
        frame,
        (HOLDING_AREA[0], HOLDING_AREA[1]),
        (HOLDING_AREA[2], HOLDING_AREA[3]),
        (255, 0, 0),  # 蓝色边框
        2
    )

    # 如果检测到球体，显示其位置
    if state in [2, 3] and flag_found_ball and ball_data:
        ball_center = ball_data['center']
        cv2.circle(frame, (int(ball_center[0]), int(ball_center[1])), 10, (0, 0, 255), -1)

    # 显示和保存
    stream.show_frame(frame, results, draw_rect=True)
    stream.save_frame(frame)
    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()
stream.release()
controller.close()
