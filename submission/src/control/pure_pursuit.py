#!/usr/bin/env python3
"""
Pure Pursuit 조향 알고리즘 모듈

기하학적 경로 추종 알고리즘으로, 전방 주시점(lookahead point)을 향해
조향각을 계산합니다.

알고리즘:
    δ = atan2(2 * L * sin(α), ld)
    
    여기서:
    - δ: 조향각 (steering angle)
    - L: 휠베이스 (wheelbase)
    - α: 전방 주시점까지의 방위각 (heading to lookahead point)
    - ld: 전방 주시 거리 (lookahead distance)

참고문헌:
    [1] Coulter, R.C. (1992). "Implementation of the Pure Pursuit Path Tracking
        Algorithm". CMU-RI-TR-92-01, Carnegie Mellon University.
    [2] Snider, J.M. (2009). "Automatic Steering Methods for Autonomous
        Automobile Path Tracking". CMU-RI-TR-09-08.

Author: HYCU Autonomous Driving Team
"""
from math import atan2, sqrt
from typing import Dict, List, Optional

import numpy as np


def get_lookahead_point(
    centerline: List[Dict[str, float]],
    current_speed: float,
    lookahead_base: float = 4.0,
    lookahead_speed_gain: float = 0.5
) -> Optional[Dict[str, float]]:
    """Arc-length 기반 전방 주시점 선택.
    
    속도에 비례하여 lookahead 거리를 조정하고, arc-length를 따라
    해당 거리에 있는 점을 반환합니다.
    
    Args:
        centerline: 중앙선 포인트 리스트 [{'x': float, 'y': float}, ...]
        current_speed: 현재 차량 속도 (m/s)
        lookahead_base: 기본 전방 주시 거리 (m)
        lookahead_speed_gain: 속도당 추가 거리 (m/(m/s))
    
    Returns:
        전방 주시점 {'x': float, 'y': float} 또는 None
    """
    if not centerline:
        return None
    
    lookahead = lookahead_base + lookahead_speed_gain * current_speed
    
    arc_length = 0.0
    prev = {'x': 0.0, 'y': 0.0}
    
    for point in centerline:
        segment = sqrt((point['x'] - prev['x'])**2 + (point['y'] - prev['y'])**2)
        arc_length += segment
        if arc_length >= lookahead:
            return point
        prev = point
    
    return centerline[-1] if centerline else None


def pure_pursuit_steering(
    target: Optional[Dict[str, float]],
    wheelbase: float = 1.5,
    max_steering: float = 0.4
) -> float:
    """Pure Pursuit 조향각 계산.
    
    전방 주시점을 향한 조향각을 기하학적으로 계산합니다.
    
    Args:
        target: 전방 주시점 {'x': float, 'y': float}
        wheelbase: 차량 휠베이스 (m)
        max_steering: 최대 조향각 (rad)
    
    Returns:
        조향각 (rad), [-max_steering, max_steering] 범위로 클리핑
    """
    if target is None:
        return 0.0
    
    alpha = atan2(target['y'], target['x'])
    lookahead_dist = sqrt(target['x']**2 + target['y']**2)
    
    if lookahead_dist < 0.1:
        return 0.0
    
    steering = atan2(2 * wheelbase * np.sin(alpha), lookahead_dist)
    return float(np.clip(steering, -max_steering, max_steering))
