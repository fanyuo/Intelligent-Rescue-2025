# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: main.py
# 功能: 工程实践与创新能力大赛智能救援赛项
# 团队: 北京建筑大学314工作室
# 创建日期: 2025-07-04
# 修改记录:
#   2025-07-05 v1.0.0 初始版本 樊彧创建并完成基本架构编写
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
import numpy as np
import time
from control import Controller
from vison import VideoStream
from vison import yolo_detect,has_caught_ball,is_ball_in_area


stream = VideoStream()
controller = Controller()
model = YOLO('xxx.pt') #<------------------------------模型文件修改此处
model.to('cuda:0')  # 使用GPU推理



balls=["red_ball","blue_ball","yellow_ball","black_ball"]
areas=[]
state: int = 0
state_list=["执行寻找球","已找到球，执行抓球","已抓到球，执行寻找area","已找到area，执行放球和后续操作"]
print("准备就绪")

# 初始化参数
frame_count = 0
detect_interval = 1  # 每x帧进行一次检测
results = None  # 存储模型检测结果
running=True
while running:

    print(f"当前State：{state_list[state]}")

    # 读取帧
    frame = stream.read_frame()

    frame_count += 1

    # 执行寻找球
    if state==0:

        if frame_count % detect_interval == 0:
            results = model(frame)
            flag_found_ball, ball_data = yolo_detect(results, model, balls)
            # 解包ball_date 获取坐标数据等

            if flag_found_ball:
                x, y = ball_data['center']
                # 在爪子内，执行抓球
                if 0<x<640 and y<480: # 坐标待定
                    state=1
                else:
                    controller.approach_ball(x, y) #待写

            else:
                controller.search_ball()   # 没找到球，旋转车体进行找球

    # 已找到球，执行抓球
    elif state==1:
        # 球已在抓取范围内，执行抓取动作
        controller.catch()

        # 设置超时时间为1秒，但条件满足时立即退出
        start_time = time.time()
        while time.time() - start_time < 1.0:  # 1秒超时检测
            if has_caught_ball():  # 成功检测到抓取
                state = 2
                break
            time.sleep(0.05)  # 短暂休眠避免CPU满载
        else:  # while循环正常结束（未触发break）
            state = 0  # 超时未抓到，返回状态0

    # 已抓到球，执行寻找area
    if state==2:
        if frame_count % detect_interval == 0:
            results = model(frame)
            flag_found_area, area_data = yolo_detect(results, model, ["area"])
            if flag_found_area:
                x, y = area_data['center']
                if is_ball_in_area():# 爪子的机械结构满足可以在不张开爪子的情况下直接将球推进安全区
                    state=3
                    continue
                if 0 < x < 640 and y < 480:  # 坐标待定
                    state = 3
                else:
                    controller.approach_area(x,y)
                    # 执行找安全区
            else:
                controller.search_area()  # 没找到球，旋转车体进行找球


    # 已到达area，执行放球和后续操作
    if state==3:
        controller.release()
        time.sleep(0.5) #参数待定

        controller.backward(10) #参数待定
        time.sleep(0.5) #参数待定

        state=0