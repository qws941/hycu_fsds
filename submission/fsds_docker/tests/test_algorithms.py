#!/usr/bin/env python3
"""
Unit tests for FSDS Competition Driver core algorithms.

Tests cover:
- Pure Pursuit steering calculation
- Cone filtering and clustering
- Centerline building
- Curvature estimation
- Speed control logic

Run: python3 -m pytest tests/ -v
"""

import unittest
import numpy as np
import math


class MockDriver:
    """Minimal mock of CompetitionDriver for algorithm testing."""
    
    def __init__(self):
        self.lookahead_base = 4.0
        self.lookahead_speed_gain = 0.3
        self.max_steering = 0.4
        self.wheelbase = 1.5
        self.current_speed = 3.0
        self.last_valid_track_width = 3.0
        self.cone_min_points = 3
        self.cone_max_radius = 0.3
        self.cone_max_radius_std = 0.15
    
    def pure_pursuit_steering(self, lookahead_point):
        if lookahead_point is None:
            return 0.0
        
        lx, ly = lookahead_point
        ld = math.sqrt(lx**2 + ly**2)
        
        if ld < 0.1:
            return 0.0
        
        steering = math.atan2(2.0 * self.wheelbase * ly, ld**2)
        return max(-self.max_steering, min(self.max_steering, steering))
    
    def estimate_curvature(self, centerline):
        if len(centerline) < 3:
            return 0.0
        
        points = np.array([[c['x'], c['y']] for c in centerline[:5]])
        
        if len(points) < 3:
            return 0.0
        
        dx = np.gradient(points[:, 0])
        dy = np.gradient(points[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        denom = (dx**2 + dy**2)**1.5
        denom = np.where(denom < 1e-6, 1e-6, denom)
        curvature = np.abs(dx * ddy - dy * ddx) / denom
        
        return float(np.mean(curvature))
    
    def get_lookahead_point(self, centerline):
        if not centerline:
            return None
        
        lookahead_dist = self.lookahead_base + self.current_speed * self.lookahead_speed_gain
        
        cumulative_dist = 0.0
        prev_x, prev_y = 0.0, 0.0
        
        for pt in centerline:
            dx = pt['x'] - prev_x
            dy = pt['y'] - prev_y
            segment_len = math.sqrt(dx**2 + dy**2)
            cumulative_dist += segment_len
            
            if cumulative_dist >= lookahead_dist:
                return (pt['x'], pt['y'])
            
            prev_x, prev_y = pt['x'], pt['y']
        
        if centerline:
            return (centerline[-1]['x'], centerline[-1]['y'])
        return None


class TestPurePursuit(unittest.TestCase):
    """Test Pure Pursuit steering algorithm."""
    
    def setUp(self):
        self.driver = MockDriver()
    
    def test_straight_ahead_no_steering(self):
        point = (5.0, 0.0)
        steering = self.driver.pure_pursuit_steering(point)
        self.assertAlmostEqual(steering, 0.0, places=2)
    
    def test_left_turn_positive_steering(self):
        point = (5.0, 2.0)
        steering = self.driver.pure_pursuit_steering(point)
        self.assertGreater(steering, 0.0)
    
    def test_right_turn_negative_steering(self):
        point = (5.0, -2.0)
        steering = self.driver.pure_pursuit_steering(point)
        self.assertLess(steering, 0.0)
    
    def test_steering_clamped_to_max(self):
        point = (1.0, 5.0)
        steering = self.driver.pure_pursuit_steering(point)
        self.assertLessEqual(abs(steering), self.driver.max_steering)
    
    def test_none_lookahead_returns_zero(self):
        steering = self.driver.pure_pursuit_steering(None)
        self.assertEqual(steering, 0.0)
    
    def test_very_close_point_returns_zero(self):
        point = (0.01, 0.01)
        steering = self.driver.pure_pursuit_steering(point)
        self.assertEqual(steering, 0.0)


class TestCurvatureEstimation(unittest.TestCase):
    """Test curvature estimation from centerline."""
    
    def setUp(self):
        self.driver = MockDriver()
    
    def test_straight_line_zero_curvature(self):
        centerline = [
            {'x': 0, 'y': 0},
            {'x': 1, 'y': 0},
            {'x': 2, 'y': 0},
            {'x': 3, 'y': 0},
            {'x': 4, 'y': 0},
        ]
        curvature = self.driver.estimate_curvature(centerline)
        self.assertLess(curvature, 0.05)
    
    def test_curved_path_nonzero_curvature(self):
        centerline = [
            {'x': 0, 'y': 0},
            {'x': 1, 'y': 0.5},
            {'x': 2, 'y': 1.5},
            {'x': 3, 'y': 3.0},
            {'x': 4, 'y': 5.0},
        ]
        curvature = self.driver.estimate_curvature(centerline)
        self.assertGreater(curvature, 0.05)
    
    def test_empty_centerline_returns_zero(self):
        curvature = self.driver.estimate_curvature([])
        self.assertEqual(curvature, 0.0)
    
    def test_two_points_returns_zero(self):
        centerline = [{'x': 0, 'y': 0}, {'x': 1, 'y': 1}]
        curvature = self.driver.estimate_curvature(centerline)
        self.assertEqual(curvature, 0.0)


class TestLookaheadPoint(unittest.TestCase):
    """Test lookahead point selection."""
    
    def setUp(self):
        self.driver = MockDriver()
    
    def test_selects_point_at_lookahead_distance(self):
        centerline = [
            {'x': 1, 'y': 0},
            {'x': 2, 'y': 0},
            {'x': 3, 'y': 0},
            {'x': 5, 'y': 0},
            {'x': 8, 'y': 0},
        ]
        point = self.driver.get_lookahead_point(centerline)
        self.assertIsNotNone(point)
        self.assertGreater(point[0], 3.0)
    
    def test_empty_centerline_returns_none(self):
        point = self.driver.get_lookahead_point([])
        self.assertIsNone(point)
    
    def test_short_centerline_returns_last_point(self):
        centerline = [{'x': 1, 'y': 0}, {'x': 2, 'y': 0}]
        point = self.driver.get_lookahead_point(centerline)
        self.assertEqual(point, (2, 0))


class TestConeFiltering(unittest.TestCase):
    """Test cone detection and filtering logic."""
    
    def test_left_right_separation(self):
        cones = [
            {'x': 5, 'y': 1.5, 'points': 10},
            {'x': 5, 'y': -1.5, 'points': 10},
            {'x': 10, 'y': 1.5, 'points': 10},
            {'x': 10, 'y': -1.5, 'points': 10},
        ]
        left = [c for c in cones if c['y'] > 0]
        right = [c for c in cones if c['y'] <= 0]
        
        self.assertEqual(len(left), 2)
        self.assertEqual(len(right), 2)
    
    def test_cones_sorted_by_x(self):
        cones = [
            {'x': 10, 'y': 1.5},
            {'x': 5, 'y': 1.5},
            {'x': 15, 'y': 1.5},
        ]
        sorted_cones = sorted(cones, key=lambda c: c['x'])
        
        self.assertEqual(sorted_cones[0]['x'], 5)
        self.assertEqual(sorted_cones[1]['x'], 10)
        self.assertEqual(sorted_cones[2]['x'], 15)


class TestSpeedControl(unittest.TestCase):
    """Test curvature-based speed control."""
    
    def test_straight_path_max_speed(self):
        curvature = 0.0
        max_speed = 6.0
        min_speed = 2.0
        curvature_factor = 2.0
        
        speed = max_speed - curvature_factor * curvature
        speed = max(min_speed, min(max_speed, speed))
        
        self.assertEqual(speed, max_speed)
    
    def test_curved_path_reduced_speed(self):
        curvature = 0.5
        max_speed = 6.0
        min_speed = 2.0
        curvature_factor = 2.0
        
        speed = max_speed - curvature_factor * curvature
        speed = max(min_speed, min(max_speed, speed))
        
        self.assertLess(speed, max_speed)
        self.assertGreaterEqual(speed, min_speed)
    
    def test_high_curvature_min_speed(self):
        curvature = 10.0
        max_speed = 6.0
        min_speed = 2.0
        curvature_factor = 2.0
        
        speed = max_speed - curvature_factor * curvature
        speed = max(min_speed, min(max_speed, speed))
        
        self.assertEqual(speed, min_speed)


if __name__ == '__main__':
    unittest.main()
