#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# 文件: vision.py
# 功能: 视觉处理核心模块
# 团队: 北京建筑大学工程设计创新中心314工作室
# 创建日期: 2025-07-05
# 修改记录:
#   2025-07-05 v1.0.0 初始版本 樊彧创建并完成基本架构编写
#   2025-07-05 樊彧 将视觉识别部分整合成类
#   2025-07-14 樊彧 优化有效球的识别，添加安全区的识别
#   2025-07-18 樊彧 增加检测框绘制
# -----------------------------------------------------------------------------
import cv2
from typing import Tuple, Optional, Dict, Any, List
import numpy as np
import time
from pathlib import Path

from config import (
    VIDEO_OUTPUT_DIR,
    SAVE_OUTPUT,
    VIDEO_CODEC,
    DEFAULT_FPS,
    DEFAULT_RESOLUTION,
    CATCH_AREA,
    READY_AREA
)

class VideoStream:
    def __init__(
            self,
            src=0,
            width: int = DEFAULT_RESOLUTION[0],
            height: int = DEFAULT_RESOLUTION[1],
            fps: int = DEFAULT_FPS,
            save_output: Optional[bool] = None
    ):
        # 确定是否保存输出
        self.save_output = save_output if save_output is not None else SAVE_OUTPUT
        self.window_name = "Video Stream"

        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise IOError("无法打开视频源")

        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # 初始化视频写入器
        self.writer = self._init_video_writer(width, height) if self.save_output else None

    @staticmethod
    def _init_video_writer(width: int, height: int) -> cv2.VideoWriter:
        """初始化视频写入器"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        output_path = Path(VIDEO_OUTPUT_DIR) / f"output_{timestamp}.mp4"

        fourcc = cv2.VideoWriter_fourcc(*VIDEO_CODEC)
        writer = cv2.VideoWriter(
            str(output_path),
            fourcc,
            DEFAULT_FPS,
            (width, height)
        )

        if not writer.isOpened():
            raise RuntimeError(f"无法创建视频写入器: {output_path}")

        print(f"视频输出已启用，保存路径: {output_path}")
        return writer

    def read_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("视频流读取失败")
        return frame

    @staticmethod
    def show_frame(
            frame: np.ndarray,
            results: List[Any],  # 根据实际结果类型替换Any
            *,
            draw_rect: bool = True,
            confidence_threshold: float = 0.5,
            window_name: str = "Detection",
            rect_color: Tuple[int, int, int] = (0, 255, 0),
            rect_thickness: int = 2,
            show_confidence: bool = True,
            font_scale: float = 0.6
    ) -> None:
        """
        显示带检测结果的帧画面（支持多种绘制选项）

        参数:
            frame: 输入图像帧 (BGR格式)
            results: 检测结果列表
            draw_rect: 是否绘制检测框 (默认True)
            confidence_threshold: 置信度阈值 (默认0.5)
            window_name: 显示窗口名称 (默认"Detection")
            rect_color: 框线颜色 (BGR格式，默认绿色)
            rect_thickness: 框线粗细 (像素，默认2)
            show_confidence: 是否显示置信度 (默认True)
            font_scale: 字体大小 (默认0.6)

        返回:
            None: 直接显示图像窗口
        """
        display_frame = frame.copy()

        if draw_rect and results:
            boxes = []
            confidences = []
            class_ids = []

            # 提取有效检测结果
            for r in results:
                for box in r.boxes:
                    conf = box.conf.item()
                    if conf > confidence_threshold:
                        boxes.append(box.xyxy[0].cpu().numpy().astype(int))
                        confidences.append(conf)
                        class_ids.append(int(box.cls.item()))

            # 绘制检测框和标签
            for box, conf, cls_id in zip(boxes, confidences, class_ids):
                x1, y1, x2, y2 = box

                # 绘制矩形框
                cv2.rectangle(
                    display_frame,
                    (x1, y1), (x2, y2),
                    color=rect_color,
                    thickness=rect_thickness
                )

                # 绘制标签和置信度
                label = f"{cls_id}: {conf:.2f}" if show_confidence else f"{cls_id}"
                cv2.putText(
                    display_frame,
                    label,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    rect_color,
                    max(1, int(font_scale * 2))
                )

        # 显示图像
        cv2.imshow(window_name, display_frame)
        cv2.waitKey(1)  # 允许图像窗口更新

    def save_frame(self, frame: np.ndarray):
        if self.save_output and self.writer:
            self.writer.write(frame)

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
        if self.save_output and self.writer:
            self.writer.release()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()



class VISION:
    def __init__(
            self,
            model,
            label_balls: List[str],
            label_area: List[str],
            catch_area: Tuple[int, int, int, int] = CATCH_AREA, # 去config.py里调整，不要在这里调整
            ready_area: Tuple[int, int, int, int] = READY_AREA, # 去config.py里调整，不要在这里调整
            min_confidence: float = 0.5
    ):

        self.model = model
        self.label_balls = label_balls
        self.label_area = label_area
        self.min_confidence = min_confidence

        # 初始化区域配置
        self.catch_area = catch_area
        self.ready_area = ready_area

    def detect_closest_ball(
            self,
            results,
            labels: Optional[List[str]] = None,
            validity = True, # 是否检测安全区内的球
            min_confidence: Optional[float] = None
    ) -> Tuple[bool, Optional[Dict[str, Any]]]:
        """
        检测最近的目标物体

        参数:
            results: YOLO检测结果
            labels: 目标标签列表（None则使用类默认）
            min_confidence: 置信度阈值（None则使用类默认）

        返回:
            tuple: (是否检测到, 目标信息字典)
                  包含: label, confidence, coordinates, center, class_id
        """
        labels = self.label_balls if labels is None else labels
        min_confidence = self.min_confidence if min_confidence is None else min_confidence
        closest_ball = None
        max_area = 0  # 用面积作为距离代理（面积越大通常距离越近）

        area_box = None
        if validity:
            # 检查安全区是否存在
            for result in results:
                for box in result.boxes:
                    conf = box.conf[0].item()
                    cls = int(box.cls[0].item())
                    label = self.model.names[cls]
                    if label in self.label_area and conf > min_confidence:
                        area_box = box

        for result in results:
            for box in result.boxes:
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = self.model.names[cls]

                # 跳过非目标标签或低置信度检测
                if label not in labels or conf < min_confidence:
                    continue

                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                current_area = (x2 - x1) * (y2 - y1)

                if validity and area_box:
                    area_x1, area_y1, area_x2, area_y2 = area_box.xyxy[0].cpu().numpy().tolist()
                    # 检查四个角点是否都在安全区内
                    corners_inside = False
                    for x, y in [(x1, y1), (x2, y1), (x1, y2), (x2, y2)]:
                        if area_x1 <= x <= area_x2 and area_y1 <= y <= area_y2:
                            corners_inside += 1
                    # 如果全部四个角点都在安全区内，则跳过
                    if corners_inside == 4:
                        continue

                # 更新最近球体信息
                if current_area > max_area:
                    max_area = current_area
                    closest_ball = {
                        'label': label,
                        'coordinates': (x1, y1, x2, y2),
                        'center': ((x1 + x2) / 2, (y1 + y2) / 2),
                        'class_id': cls
                    }

        return closest_ball is not None, closest_ball

    def detect_area(
            self,
            results,
            labels: Optional[List[str]] = None,
            min_confidence: Optional[float] = None
    ) -> Tuple[bool, Optional[Dict[str, Any]]]:

        labels = self.label_area if labels is None else labels
        min_confidence = self.min_confidence if min_confidence is None else min_confidence

        area_box = None

        for result in results:
            for box in result.boxes:
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = self.model.names[cls]

                if label in labels and conf > min_confidence:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()

                    area_box = {
                        'label': label,
                        'coordinates': (x1, y1, x2, y2),
                        'center': ((x1 + x2) / 2, (y1 + y2) / 2),
                        'class_id': cls
                    }

        return area_box is not None, area_box

    def has_caught_ball(
            self,
            results,
            labels: Optional[List[str]] = None,
            min_confidence: Optional[float] = None
    ) -> bool:
        """
        判断是否成功抓取到目标球体

        参数:
            results: YOLO检测结果
            labels: 目标标签列表（None则使用类默认）
            min_confidence: 置信度阈值（None则使用类默认）

        返回:
            bool: 是否检测到目标且在抓取区域内
        """
        # 解包抓取区域坐标
        catch_x1, catch_y1, catch_x2, catch_y2 = self.catch_area
        labels = self.label_balls if labels is None else labels
        min_confidence = self.min_confidence if min_confidence is None else min_confidence

        for result in results:
            for box in result.boxes:
                # 获取检测信息
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = self.model.names[cls]

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

    # 此方法不靠谱，考虑以下方案：只推球不考虑是否推进去
    def is_ready_to_release(
            self,
            results,
            min_confidence: Optional[float] = None
    ) -> bool:
        """
        严格检查球体是否从抓取区域进入安全区

        参数:
            results: YOLO检测结果
            min_confidence: 置信度阈值（None则使用类默认）

        返回:
            bool: 是否满足"先进入抓取区域，再到达安全区"
        """
        # 解包抓取区域坐标
        catch_x1, catch_y1, catch_x2, catch_y2 = self.catch_area
        min_confidence = self.min_confidence if min_confidence is None else min_confidence

        # 第一阶段：检查安全区是否存在
        area_box = None

        for result in results:
            for box in result.boxes:
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = self.model.names[cls]
                if label in self.label_area and conf > min_confidence:
                    area_box=box

        if not area_box:
            return False

        area_x1, area_y1, area_x2, area_y2 = area_box.xyxy[0].cpu().numpy().tolist()

        # 第二阶段：检查球体位置
        for result in results:
            for box in result.boxes:
                # 获取球体信息
                conf = box.conf[0].item()
                cls = int(box.cls[0].item())
                label = self.model.names[cls]

                if label in self.label_balls and conf > min_confidence:
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

    def is_ready_to_catch(self, x: float, y: float) -> bool:
        """
        判断是否处于准备抓取状态

        参数:
            x: 目标x坐标
            y: 目标y坐标

        返回:
            bool: 是否在准备区域内
        """
        ready_x1, ready_y1, ready_x2, ready_y2 = self.ready_area
        return ready_x1 <= x <= ready_x2 and ready_y1 <= y <= ready_y2


# ===================== 测试程序 =====================

def main_yolo():
    import cv2
    from ultralytics import YOLO
    from config import RESCUE_MODEL

    model = YOLO(str(RESCUE_MODEL))
    # model.to('cuda:0')  # 使用GPU推理
    stream = VideoStream()

    # 初始化参数
    frame_count = 0
    detect_interval = 1  # 每x帧进行一次检测
    results = None  # 存储模型检测结果

    while True:
        try:
            # 读取帧
            frame = stream.read_frame()
            frame_count += 1

            if frame_count % detect_interval == 0:
                results = model(frame, verbose=False)

            # 显示和保存
            stream.show_frame(frame, results, draw_rect=True)
            stream.save_frame(frame)
            if cv2.waitKey(1) == ord('q'):
                break

        except KeyboardInterrupt:
            break

    stream.release()

if __name__ == '__main__':
    main_yolo()

    