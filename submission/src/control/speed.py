#!/usr/bin/env python3
"""
속도 제어 모듈

곡률 기반 속도 제어 및 rate limiting 기능을 제공합니다.

알고리즘:
    v = √(a_lat_max / κ)
    
    여기서:
    - v: 목표 속도 (m/s)
    - a_lat_max: 최대 허용 횡가속도 (m/s²)
    - κ: 경로 곡률 (1/m)

참고문헌:
    [1] Rajamani, R. (2011). "Vehicle Dynamics and Control".
        Springer, Chapter 2: Lateral Vehicle Dynamics.

Author: HYCU Autonomous Driving Team
"""
from typing import List, Dict

import numpy as np


def estimate_curvature(
    centerline: List[Dict[str, float]],
    curvature_history: List[float],
    smooth_window: int = 5
) -> float:
    """Arc-length 파라미터화를 사용한 경로 곡률 추정.
    
    불규칙한 콘 간격을 올바르게 처리하기 위해 arc-length 기반
    미분을 사용합니다.
    
    곡률 공식: κ = |dx/ds * d²y/ds² - dy/ds * d²x/ds²|
    여기서 s는 arc-length 파라미터입니다.
    
    Args:
        centerline: 중앙선 포인트 리스트
        curvature_history: 이전 곡률 값 리스트 (smoothing용, 수정됨)
        smooth_window: 평활화 윈도우 크기
    
    Returns:
        평활화된 곡률 값
    """
    if len(centerline) < 3:
        return _get_smoothed_curvature(0.0, curvature_history, smooth_window)
    
    points = np.array([[c['x'], c['y']] for c in centerline[:5]])
    
    if len(points) < 3:
        return _get_smoothed_curvature(0.0, curvature_history, smooth_window)
    
    # Compute arc-length between consecutive points
    diffs = np.diff(points, axis=0)
    segment_lengths = np.sqrt(np.sum(diffs**2, axis=1))
    
    # Avoid division by zero for coincident points
    segment_lengths = np.maximum(segment_lengths, 1e-6)
    
    # Arc-length parameterized derivatives: dx/ds, dy/ds
    dx_ds = diffs[:, 0] / segment_lengths
    dy_ds = diffs[:, 1] / segment_lengths
    
    if len(dx_ds) < 2:
        return _get_smoothed_curvature(0.0, curvature_history, smooth_window)
    
    # Second derivatives: d²x/ds², d²y/ds²
    mid_lengths = (segment_lengths[:-1] + segment_lengths[1:]) / 2
    mid_lengths = np.maximum(mid_lengths, 1e-6)
    
    ddx_ds = np.diff(dx_ds) / mid_lengths
    ddy_ds = np.diff(dy_ds) / mid_lengths
    
    # Curvature: κ = |dx/ds * d²y/ds² - dy/ds * d²x/ds²|
    dx_mid = (dx_ds[:-1] + dx_ds[1:]) / 2
    dy_mid = (dy_ds[:-1] + dy_ds[1:]) / 2
    
    curvature = np.abs(dx_mid * ddy_ds - dy_mid * ddx_ds)
    raw_curvature = float(np.mean(curvature)) if len(curvature) > 0 else 0.0
    
    return _get_smoothed_curvature(raw_curvature, curvature_history, smooth_window)


def _get_smoothed_curvature(
    new_curvature: float,
    curvature_history: List[float],
    smooth_window: int
) -> float:
    """곡률 평활화 (moving average).
    
    Args:
        new_curvature: 새로운 곡률 값
        curvature_history: 이전 값 리스트 (수정됨)
        smooth_window: 윈도우 크기
    
    Returns:
        평활화된 곡률
    """
    curvature_history.append(new_curvature)
    if len(curvature_history) > smooth_window:
        curvature_history.pop(0)
    return float(np.mean(curvature_history))


def calculate_target_speed(
    curvature: float,
    max_lateral_accel: float = 4.0,
    min_speed: float = 2.0,
    max_speed: float = 6.0,
    v2x_speed_limit: float = 6.0,
    v2x_hazard: bool = False
) -> float:
    """곡률 기반 목표 속도 계산.
    
    횡가속도 제한 모델을 사용하여 커브에서 안전한 속도를 계산합니다.
    V2X 속도 제한 및 위험 경고도 반영합니다.
    
    Args:
        curvature: 경로 곡률 (1/m)
        max_lateral_accel: 최대 횡가속도 (m/s²)
        min_speed: 최소 속도 (m/s)
        max_speed: 최대 속도 (m/s)
        v2x_speed_limit: V2X 속도 제한 (m/s)
        v2x_hazard: V2X 위험 경고 플래그
    
    Returns:
        목표 속도 (m/s)
    """
    speed = np.sqrt(max_lateral_accel / (abs(curvature) + 0.01))
    speed = np.clip(speed, min_speed, max_speed)
    
    if v2x_speed_limit <= 0:
        return 0.0
    speed = min(speed, v2x_speed_limit)
    speed = max(0.0, speed)
    
    if v2x_hazard:
        speed = min(speed, min_speed)
    
    if not np.isfinite(speed):
        return min_speed
    return float(speed)


def apply_rate_limit(
    target: float,
    current: float,
    max_rate_per_sec: float,
    dt: float
) -> float:
    """Rate limiting 적용.
    
    급격한 제어 입력 변화를 방지하기 위해 변화율을 제한합니다.
    
    Args:
        target: 목표 값
        current: 현재 값
        max_rate_per_sec: 초당 최대 변화율
        dt: 시간 간격 (초)
    
    Returns:
        Rate-limited 값
    """
    max_delta = max_rate_per_sec * dt
    delta = target - current
    if abs(delta) > max_delta:
        return current + max_delta * np.sign(delta)
    return target


def calculate_throttle(
    target_speed: float,
    current_speed: float,
    max_throttle: float = 0.25
) -> float:
    """스로틀 계산 (P 제어).
    
    Args:
        target_speed: 목표 속도 (m/s)
        current_speed: 현재 속도 (m/s)
        max_throttle: 최대 스로틀 값
    
    Returns:
        스로틀 값 [0, max_throttle]
    """
    speed_error = target_speed - current_speed
    throttle = 0.3 * speed_error
    return float(np.clip(throttle, 0.0, max_throttle))
