#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: vision.py
# 功能: 视觉处理核心模块
# 团队: 北京建筑大学工程设计创新中心314工作室
# 创建日期: 2025-07-05
# 修改记录:
#   2025-07-05 v1.0.0 初始版本 樊彧创建并完成基本架构编写
# -----------------------------------------------------------------------------
import cv2
from typing import Tuple, Optional, Dict, Any
import numpy as np

from config import label_balls,label_areas

class VideoStream:
    def __init__(
            self,
            src=0,
            width: int = 640,
            height: int = 480,
            fps: int = 30
    ):
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise IOError("无法打开视频源")

        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)


    def read_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("视频流读取失败")
        return frame

    def release(self):
        self.cap.release()



def detect_closest_object(
        results,
        model,
        labels: list,
        min_confidence: float = 0.5
) -> Tuple[bool, Optional[Dict[str, Any]]]:
    """
    YOLO目标检测函数（返回最近目标）

    参数:
        results: YOLO检测结果
        model: YOLO模型对象
        labels: 需要检测的目标类别列表
        min_confidence: 最小置信度阈值

    返回:
        tuple: (是否检测到, 目标信息字典)
              目标信息包含: label, confidence, coordinates, center, area
    """
    closest_target = None
    max_area = 0  # 用面积作为距离代理（面积越大通常距离越近）

    for result in results:
        for box in result.boxes:
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            label = model.names[cls]

            if label in labels and conf > min_confidence:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                current_area = (x2 - x1) * (y2 - y1)

                # 只保留面积最大的目标（最近的）
                if current_area > max_area:
                    max_area = current_area
                    closest_target = {
                        'label': label,
                        'coordinates': (x1, y1, x2, y2),
                        'center': ((x1 + x2) / 2, (y1 + y2) / 2),
                        'class_id': cls
                    }

    return closest_target is not None, closest_target


catch_area: tuple = (300, 200, 340, 280)  # (x1, y1, x2, y2) <------------------------修改此处

def has_caught_ball(
        results,
        model,
        labels: list = label_balls,
        catch_area_box: tuple = catch_area,
        min_confidence: float = 0.5,
) -> bool:
    """
    判断是否成功抓取到目标球体

    参数:
        results: YOLO检测结果对象
        model: YOLO模型实例
        labels: 目标球体类别列表
        min_confidence: 最小置信度阈值
        catch_area: 抓取区域坐标 (x1, y1, x2, y2)

    返回:
        bool: 是否检测到目标且在抓取区域内
    """
    # 解包抓取区域坐标
    catch_x1, catch_y1, catch_x2, catch_y2 = catch_area_box

    for result in results:
        for box in result.boxes:
            # 获取检测信息
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            label = model.names[cls]

            if label in labels and conf > min_confidence:
                # 解包并计算中心坐标
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                ball_center_x = (x1 + x2) / 2
                ball_center_y = (y1 + y2) / 2

                # 判断中心点是否在抓取区域内
                is_in_catch_area = (catch_x1 <= ball_center_x <= catch_x2 and
                                    catch_y1 <= ball_center_y <= catch_y2)

                if is_in_catch_area:
                    return True
    return False


def is_ball_in_area(
        results,
        model,
        min_confidence: float = 0.5,
        catch_area_box: tuple = catch_area
)->bool:
    """
    严格检查球体是否从抓取区域进入安全区

    参数:
        results: YOLO检测结果
        model: YOLO模型
        min_confidence: 置信度阈值
        catch_area_box: 抓取区域坐标(x1,y1,x2,y2)

    返回:
        bool: 是否满足"先进入抓取区域，再到达安全区"
    """
    # 解包抓取区域坐标
    catch_x1, catch_y1, catch_x2, catch_y2 = catch_area_box

    # 第一阶段：检查安全区是否存在
    area_box = next(
        (box for result in results for box in result.boxes
         if model.names[int(box.cls[0].item())] in label_areas
         and box.conf[0].item() > min_confidence),
        None
    )
    if not area_box:
        return False

    # 解包安全区坐标
    area_x1, area_y1, area_x2, area_y2 = area_box.xyxy[0].cpu().numpy().tolist()

    # 第二阶段：检查球体位置
    for result in results:
        for box in result.boxes:
            # 获取球体信息
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            label = model.names[cls]

            if label in ["red_ball", "blue_ball"] and conf > min_confidence:
                # 计算球体中心
                ball_x1, ball_y1, ball_x2, ball_y2 = box.xyxy[0].cpu().numpy().tolist()
                center_x, center_y = (ball_x1 + ball_x2) / 2, (ball_y1 + ball_y2) / 2

                # 双重条件检查
                in_catch = (catch_x1 <= center_x <= catch_x2 and
                            catch_y1 <= center_y <= catch_y2)
                in_area = (area_x1 < center_x < area_x2 and
                           area_y1 < center_y < area_y2)

                if in_catch and in_area:
                    return True
    return False


