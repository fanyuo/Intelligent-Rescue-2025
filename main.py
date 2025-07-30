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
from control import Controller
from vision import VideoStream,VISION
from config import label_balls,label_area,RESCUE_MODEL

model = YOLO(str(RESCUE_MODEL))
model.to('cuda:0')  # 使用GPU推理

stream = VideoStream()
controller = Controller()
vision=VISION(model,label_balls,label_area)

state: int = 0
state_list=["执行寻找球","已找到球，执行抓球","已抓到球，执行寻找area","已找到area，执行放球和后续操作"]
print("准备就绪")

# 初始化参数
frame_count = 0
detect_interval = 1  # 每x帧进行一次检测
start_time=None
results = None  # 存储模型检测结果
running=True
last_flag_found_ball=False
last_flag_found_area=False
last_time = time.time()

while running:

    # 读取帧
    frame = stream.read_frame()
    frame_count += 1

    if frame_count % detect_interval == 0:
        results = model(frame, stream=True, verbose=False)

    # 每0.5s打印一次当前状态
    if (current_time := time.time()) > last_time + 0.5:
        last_time=current_time
        print(f"第{frame_count}次运行。当前状态：{state_list[state]}")

    # 执行寻找球
    if state==0:
        flag_found_ball, ball_data = vision.detect_closest_ball(results,label_balls)

        if flag_found_ball:
            if not last_flag_found_ball:
                last_flag_found_ball=True
                controller.stop()

            # 解包ball_date 获取坐标数据等
            bcx, bcy = ball_data['center']
            # 在爪子内，执行抓球
            if vision.is_ready_to_catch(bcx,bcy):
                state=1
            else:
                controller.approach_ball(bcx, bcy)
        else:
            controller.search_ball(speed=6)   # 没找到球，旋转车体进行找球
            last_flag_found_ball=False

    # 已找到球，执行抓球
    if state==1:
        # 球已在抓取范围内，执行抓取动作
        controller.catch()

        # # 设置超时时间为1秒，但条件满足时立即退出
        # start_time = time.time()
        # while time.time() - start_time < 1.0:  # 超时检测
        #     frame = stream.read_frame()
        #     results = model(frame,verbose=False)
        #     if vision.has_caught_ball(results):  # 成功检测到抓取
        #         state = 2
        #         break
        #     time.sleep(0.05)  # 短暂休眠避免CPU满载
        # else:  # while循环正常结束（未触发break）
        #     state = 0  # 超时未抓到，返回状态0

        if not start_time:
            start_time = time.time()
        elif time.time() - start_time <= 1.0 and vision.has_caught_ball(results):
            start_time = None
            state = 2
        elif time.time() - start_time > 1.0:
            start_time = None
            state = 0


    # 已抓到球，执行寻找area
    if state==2:
        flag_found_area, area_data = vision.detect_area(results)

        if flag_found_area:
            if not last_flag_found_area:
                last_flag_found_area=True
                controller.stop()

            acx, acy = area_data['center']
            if vision.is_ready_to_release(results): # 爪子的机械结构满足可以在不张开爪子的情况下直接将球推进安全区
                state=3
                continue
            else:
                controller.approach_area(acx,acy)
        # 执行找安全区
        else:
            controller.search_area(speed=7)  # 没找到球，旋转车体进行找球
            last_flag_found_area = False

    # 已到达area，执行放球和后续操作
    if state==3:
        controller.release()
        time.sleep(0.5) #参数待定

        controller.backward(speed=8) #参数待定
        time.sleep(0.5) #参数待定

        state=0

    # 显示和保存
    stream.show_frame(frame,results,draw_rect=True)
    stream.save_frame(frame)
    if cv2.waitKey(1) == ord('q'):
        break


stream.release()
controller.close()