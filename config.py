# config.py
"""
配置中心

本模块提供全局配置参数，同时保持与现有接口的兼容性。外部模块可通过以下属性访问配置：

属性：
    team (str): 当前选择的队伍，必须是 'blue' 或 'red'
    rescue_model (str): 模型权重文件路径
    label_balls (list): 根据当前队伍过滤后的球体标签列表
    label_areas (list): 根据当前队伍过滤后的区域标签列表
"""

from typing import Dict, List, Literal

# ==================== 用户可配置参数 ====================
team: Literal["blue", "red"] = ["blue", "red"][0]  # 当前队伍（默认选第一个）
rescue_model: str = "323.pt"                      # 模型文件路径


# ==================== 内部数据处理 ====================
_origin_label_balls = ["red_ball", "blue_ball", "yellow_ball", "black_ball"]
_origin_label_areas = ["blue_area", "red_area"]

_ball_filter = {"blue": "red_ball", "red": "blue_ball"}
_area_filter = {"blue": "red_area", "red": "blue_area"}

label_balls = [x for x in _origin_label_balls if x != _ball_filter.get(team, "")]
label_areas = [x for x in _origin_label_areas if x != _area_filter.get(team, "")]

def _validate():
    """参数安全检查"""
    assert team in ("blue", "red"), "队伍必须是 'blue' 或 'red'"
    assert isinstance(rescue_model, str), "模型路径必须是字符串"

# 初始化时自动校验
_validate()