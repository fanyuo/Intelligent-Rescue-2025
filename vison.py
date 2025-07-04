#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: vision.py
# 功能: 视觉处理核心模块
# 团队: 北京建筑大学314工作室
# 创建日期: 2025-07-05
# 修改记录:
#   2025-07-05 v1.0.0 初始版本 樊彧创建并完成基本架构编写
# -----------------------------------------------------------------------------
import cv2
from typing import Tuple, Optional, Dict, Any
import numpy as np


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



def yolo_detect(
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

def has_caught_ball()->bool:
    pass

def is_ball_in_area()->bool:
    pass # 判断爪子收起范围内的球是否在紫色区域内？