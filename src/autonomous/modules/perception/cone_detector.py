#!/usr/bin/env python3
"""
콘 검출 및 중앙선 생성 모듈

LiDAR 포인트클라우드에서 콘을 검출하고, 좌/우 콘 쌍으로부터
주행 경로 중앙선을 생성합니다.

알고리즘:
    1. Z-height 필터: 지면(-0.3m) ~ 콘 상단(0.5m) 범위만 추출
    2. Grid 기반 클러스터링 O(N): 인접 셀 BFS로 포인트 그룹화
    3. 품질 검증: 클러스터 반경, 분산, 최소 포인트 수 확인
    4. 좌/우 분리: y > 0은 좌측, y <= 0은 우측

참고문헌:
    [1] Thrun, S. et al. (2005). "Probabilistic Robotics".
        MIT Press, Chapter 9: Occupancy Grid Mapping.

Author: HYCU Autonomous Driving Team
"""
from collections import deque
from typing import Dict, List, Tuple

import numpy as np


def find_cones_filtered(
    points: np.ndarray,
    cone_min_z: float = -0.3,
    cone_max_z: float = 0.5,
    cones_range_cutoff: float = 12.0,
    cone_grouping_threshold: float = 0.2,
    cone_min_points: int = 3,
    cone_max_radius: float = 0.3,
    cone_max_radius_std: float = 0.15
) -> Tuple[List[Dict], List[Dict]]:
    """LiDAR 포인트클라우드에서 콘 검출.
    
    Grid 기반 BFS 클러스터링으로 O(N) 시간복잡도를 달성합니다.
    
    Args:
        points: LiDAR 포인트 배열 (N, 3) - x, y, z
        cone_min_z: 콘 최소 높이 (m)
        cone_max_z: 콘 최대 높이 (m)
        cones_range_cutoff: 최대 검출 범위 (m)
        cone_grouping_threshold: 그리드 셀 크기 (m)
        cone_min_points: 클러스터 최소 포인트 수
        cone_max_radius: 클러스터 최대 평균 반경 (m)
        cone_max_radius_std: 클러스터 반경 최대 표준편차 (m)
    
    Returns:
        (left_cones, right_cones) 튜플
        각 콘: {'x': float, 'y': float, 'points': int}
    """
    if len(points) == 0:
        return [], []
    
    # Filter by z-height and range
    z_mask = (points[:, 2] > cone_min_z) & (points[:, 2] < cone_max_z)
    range_mask = np.linalg.norm(points[:, :2], axis=1) < cones_range_cutoff
    front_mask = points[:, 0] > 0.3
    filtered = points[z_mask & range_mask & front_mask]
    
    if len(filtered) == 0:
        return [], []
    
    # Grid-based clustering O(N)
    grid_res = cone_grouping_threshold
    grid: Dict[Tuple[int, int], List[int]] = {}
    for i, pt in enumerate(filtered):
        gx, gy = int(pt[0] / grid_res), int(pt[1] / grid_res)
        key = (gx, gy)
        if key not in grid:
            grid[key] = []
        grid[key].append(i)
    
    cones = []
    visited_cells: set = set()
    
    for cell_key in grid.keys():
        if cell_key in visited_cells:
            continue
        
        cluster_indices: List[int] = []
        queue: deque = deque([cell_key])
        
        while queue:
            current = queue.popleft()
            if current in visited_cells:
                continue
            if current not in grid:
                continue
            
            visited_cells.add(current)
            cluster_indices.extend(grid[current])
            
            gx, gy = current
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    neighbor = (gx + dx, gy + dy)
                    if neighbor in grid and neighbor not in visited_cells:
                        queue.append(neighbor)
        
        if len(cluster_indices) < cone_min_points:
            continue
        
        group = filtered[cluster_indices]
        xy = group[:, :2]
        center = xy.mean(axis=0)
        radii = np.linalg.norm(xy - center, axis=1)
        
        if radii.mean() > cone_max_radius:
            continue
        if radii.std() > cone_max_radius_std:
            continue
        
        cones.append({
            'x': float(center[0]),
            'y': float(center[1]),
            'points': len(cluster_indices)
        })
    
    # Split into left (y > 0) and right (y <= 0), sorted by x
    left = sorted([c for c in cones if c['y'] > 0], key=lambda c: c['x'])
    right = sorted([c for c in cones if c['y'] <= 0], key=lambda c: c['x'])
    
    if cones:
        y_values = [c['y'] for c in cones]
        print(f"[DEBUG] Total cones: {len(cones)}, Y range: [{min(y_values):.2f}, {max(y_values):.2f}]")
        print(f"[DEBUG] Left(y>0): {len(left)}, Right(y<=0): {len(right)}")
    
    return left, right


def build_centerline(
    left_cones: List[Dict],
    right_cones: List[Dict],
    last_valid_track_width: float = 4.0
) -> List[Dict[str, float]]:
    """좌/우 콘으로부터 중앙선 생성.
    
    양쪽 콘이 있으면 평균, 한쪽만 있으면 추정 트랙폭으로 보정합니다.
    
    Args:
        left_cones: 좌측 콘 리스트
        right_cones: 우측 콘 리스트
        last_valid_track_width: 마지막 유효 트랙 폭 (m)
    
    Returns:
        중앙선 포인트 리스트 [{'x': float, 'y': float}, ...]
    """
    if not left_cones and not right_cones:
        return []
    
    centerline: List[Dict[str, float]] = []
    
    if left_cones and right_cones:
        all_x = sorted(set([c['x'] for c in left_cones + right_cones]))
        
        for x in all_x[:8]:
            left_at_x = [c for c in left_cones if abs(c['x'] - x) < 2.0]
            right_at_x = [c for c in right_cones if abs(c['x'] - x) < 2.0]
            
            if left_at_x and right_at_x:
                left_y = np.mean([c['y'] for c in left_at_x])
                right_y = np.mean([c['y'] for c in right_at_x])
                center_y = (left_y + right_y) / 2.0
                centerline.append({'x': x, 'y': float(center_y)})
            elif left_at_x:
                left_y = np.mean([c['y'] for c in left_at_x])
                centerline.append({'x': x, 'y': float(left_y - last_valid_track_width / 2)})
            elif right_at_x:
                right_y = np.mean([c['y'] for c in right_at_x])
                centerline.append({'x': x, 'y': float(right_y + last_valid_track_width / 2)})
    
    elif left_cones:
        for c in left_cones[:5]:
            centerline.append({'x': c['x'], 'y': c['y'] - last_valid_track_width / 2})
    
    elif right_cones:
        for c in right_cones[:5]:
            centerline.append({'x': c['x'], 'y': c['y'] + last_valid_track_width / 2})
    
    centerline = sorted(centerline, key=lambda c: c['x'])
    
    # Smooth centerline with moving average
    if len(centerline) >= 3:
        smoothed = []
        window = 3
        for i in range(len(centerline)):
            start = max(0, i - window // 2)
            end = min(len(centerline), i + window // 2 + 1)
            avg_y = np.mean([c['y'] for c in centerline[start:end]])
            smoothed.append({'x': centerline[i]['x'], 'y': float(avg_y)})
        return smoothed
    
    return centerline


def update_track_width(
    left_cones: List[Dict],
    right_cones: List[Dict],
    current_width: float,
    min_width: float = 2.0,
    max_width: float = 6.0
) -> float:
    """트랙 폭 업데이트.
    
    좌/우 콘이 모두 있을 때 트랙 폭을 갱신합니다.
    
    Args:
        left_cones: 좌측 콘 리스트
        right_cones: 우측 콘 리스트
        current_width: 현재 트랙 폭 (m)
        min_width: 최소 트랙 폭 (m)
        max_width: 최대 트랙 폭 (m)
    
    Returns:
        갱신된 트랙 폭 (m)
    """
    if not left_cones or not right_cones:
        return current_width
    
    left_y = np.mean([c['y'] for c in left_cones[:3]])
    right_y = np.mean([c['y'] for c in right_cones[:3]])
    raw_width = abs(left_y - right_y)
    
    return float(np.clip(raw_width, min_width, max_width))
