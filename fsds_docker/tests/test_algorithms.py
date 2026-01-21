import sys
import unittest
from unittest.mock import MagicMock, patch
import numpy as np
import math
import os

# =============================================================================
# MOCK ROS ENVIRONMENT (Must be done before importing drivers)
# =============================================================================
# Mock top-level packages first
sys.modules['rospy'] = MagicMock()
sys.modules['tf'] = MagicMock()
sys.modules['tf2_ros'] = MagicMock()
sys.modules['fs_msgs'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['nav_msgs'] = MagicMock()
sys.modules['std_msgs'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()

# Mock submodules
sys.modules['tf.transformations'] = MagicMock()
sys.modules['fs_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['nav_msgs.msg'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs.point_cloud2'] = MagicMock()

# Mock specific functions used in imports
tft = MagicMock()
tft.euler_from_quaternion.return_value = (0, 0, 0)
sys.modules['tf.transformations'] = tft

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../scripts')))

from competition_driver import CompetitionDriver
from simple_slam import SimpleSLAM

class TestCompetitionDriver(unittest.TestCase):
    def setUp(self):
        # Mock get_param to return default values or specific test values
        with patch('rospy.get_param') as mock_get_param:
            def side_effect(key, default=None):
                params = {
                    '~max_throttle': 0.25,
                    '~min_speed': 2.0,
                    '~max_speed': 6.0,
                    '~wheelbase': 1.5,
                    '~lookahead_base': 4.0,
                    '~lookahead_speed_gain': 0.0, # Simplify for testing
                    '~max_lateral_accel': 4.0,
                    '~max_steering': 0.5
                }
                return params.get(key, default)
            mock_get_param.side_effect = side_effect
            
            self.driver = CompetitionDriver()
            self.driver.current_speed = 5.0

    def test_pure_pursuit_steering_straight(self):
        # Target directly ahead (x=10, y=0)
        target = {'x': 10.0, 'y': 0.0}
        steering = self.driver.pure_pursuit_steering(target)
        self.assertAlmostEqual(steering, 0.0, places=5)

    def test_pure_pursuit_steering_left(self):
        # Target to the left (x=10, y=5)
        target = {'x': 10.0, 'y': 5.0}
        steering = self.driver.pure_pursuit_steering(target)
        self.assertTrue(steering > 0) # Should steer left (positive)

    def test_pure_pursuit_steering_right(self):
        # Target to the right (x=10, y=-5)
        target = {'x': 10.0, 'y': -5.0}
        steering = self.driver.pure_pursuit_steering(target)
        self.assertTrue(steering < 0) # Should steer right (negative)

    def test_estimate_curvature_straight(self):
        # Straight line points
        centerline = [
            {'x': 0, 'y': 0},
            {'x': 1, 'y': 0},
            {'x': 2, 'y': 0},
            {'x': 3, 'y': 0},
            {'x': 4, 'y': 0}
        ]
        curvature = self.driver.estimate_curvature(centerline)
        self.assertAlmostEqual(curvature, 0.0, places=5)

    def test_estimate_curvature_circle(self):
        # Points on a circle of radius R=10
        # x = R * cos(t), y = R * sin(t)
        R = 10.0
        centerline = []
        for t in np.linspace(0, 0.5, 5):
            centerline.append({'x': R * math.cos(t), 'y': R * math.sin(t)})
        
        # Note: The gradient method is an approximation, so we allow some tolerance
        # Expected curvature = 1/R = 0.1
        # However, our driver uses a smoothing window and gradient approximation
        # We just verify it's close to 0.1 and definitely not 0
        curvature = self.driver.estimate_curvature(centerline)
        self.assertGreater(curvature, 0.05)
        self.assertLess(curvature, 0.15)

    def test_calculate_target_speed(self):
        # Max speed on straight line (curvature 0)
        speed_straight = self.driver.calculate_target_speed(0.0)
        self.assertAlmostEqual(speed_straight, self.driver.max_speed)
        
        # Reduced speed on curve (curvature 0.2, R=5m)
        # v = sqrt(a_lat / k) = sqrt(4.0 / 0.2) = sqrt(20) ~= 4.47
        speed_curve = self.driver.calculate_target_speed(0.2)
        expected = math.sqrt(4.0 / (0.2 + 0.01)) # code uses +0.01 stability
        self.assertAlmostEqual(speed_curve, expected, places=2)
        self.assertTrue(speed_curve < speed_straight)

class TestSimpleSLAM(unittest.TestCase):
    def setUp(self):
        with patch('rospy.get_param') as mock_get_param:
            def side_effect(key, default=None):
                params = {
                    '~map_resolution': 0.1,
                    '~map_size': 100
                }
                return params.get(key, default)
            mock_get_param.side_effect = side_effect
            
            self.slam = SimpleSLAM()
            self.slam.robot_x = 0.0
            self.slam.robot_y = 0.0
            self.slam.robot_yaw = 0.0

    def test_grid_conversion_origin(self):
        # Map origin is -size * res / 2 = -100 * 0.1 / 2 = -5.0
        # Robot at 0,0 should be at center of grid (50, 50)
        
        world_x = 0.0
        world_y = 0.0
        
        grid_x = int((world_x - self.slam.map_origin) / self.slam.map_resolution)
        grid_y = int((world_y - self.slam.map_origin) / self.slam.map_resolution)
        
        self.assertEqual(grid_x, 50)
        self.assertEqual(grid_y, 50)

    def test_coordinate_transform(self):
        # Robot at (10, 10), Yaw = 90 deg (pi/2)
        self.slam.robot_x = 10.0
        self.slam.robot_y = 10.0
        self.slam.robot_yaw = math.pi / 2
        
        # Lidar point at (1, 0) relative to robot (1m ahead)
        # In world frame, this should be (10, 11) (since robot faces Y axis)
        p = [1.0, 0.0]
        
        world_x = self.slam.robot_x + p[0] * math.cos(self.slam.robot_yaw) - p[1] * math.sin(self.slam.robot_yaw)
        world_y = self.slam.robot_y + p[0] * math.sin(self.slam.robot_yaw) + p[1] * math.cos(self.slam.robot_yaw)
        
        self.assertAlmostEqual(world_x, 10.0, places=5)
        self.assertAlmostEqual(world_y, 11.0, places=5)

class TestWatchdogStateMachine(unittest.TestCase):
    """Test watchdog and state machine transitions - Critical safety tests."""
    
    def setUp(self):
        with patch('rospy.get_param') as mock_get_param:
            def side_effect(key, default=None):
                params = {
                    '~max_throttle': 0.25,
                    '~min_speed': 2.0,
                    '~max_speed': 6.0,
                    '~wheelbase': 1.5,
                    '~lookahead_base': 4.0,
                    '~lookahead_speed_gain': 0.0,
                    '~max_lateral_accel': 4.0,
                    '~max_steering': 0.5,
                    '~degraded_timeout': 1.0,
                    '~stopping_timeout': 3.0,
                    '~lidar_stale_timeout': 1.0,
                    '~odom_stale_timeout': 1.0,
                    '~recovery_timeout': 1.0
                }
                return params.get(key, default)
            mock_get_param.side_effect = side_effect
            
            self.driver = CompetitionDriver()
    
    def test_cones_lost_does_not_auto_recover_via_watchdog(self):
        """CRITICAL: CONES_LOST should NOT auto-recover in check_watchdog().
        Recovery should only happen via update_state() when cones are reacquired."""
        from competition_driver import DriveState, StopReason
        
        # Set up STOPPING state due to CONES_LOST
        self.driver.state = DriveState.STOPPING
        self.driver.stop_reason = StopReason.CONES_LOST
        self.driver.recovery_start_time = None
        
        # Create mock Time objects that properly handle subtraction
        # (now - last_time).to_sec() should return 0.0 (fresh sensor data)
        mock_duration = MagicMock()
        mock_duration.to_sec.return_value = 0.0  # 0 seconds elapsed = fresh
        
        mock_now = MagicMock()
        mock_now.to_sec.return_value = 100.0
        mock_now.__sub__ = MagicMock(return_value=mock_duration)
        
        mock_lidar_time = MagicMock()
        mock_lidar_time.to_sec.return_value = 100.0
        
        mock_odom_time = MagicMock()
        mock_odom_time.to_sec.return_value = 100.0
        
        with patch('rospy.Time.now', return_value=mock_now):
            self.driver.last_lidar_time = mock_lidar_time
            self.driver.last_odom_time = mock_odom_time
            
            # Call watchdog - should NOT recover from CONES_LOST
            self.driver.check_watchdog()
        
        # State should remain STOPPING with CONES_LOST
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        self.assertEqual(self.driver.stop_reason, StopReason.CONES_LOST)
    
    def test_v2x_stop_zone_forces_stopping(self):
        """V2X stop_zone=True should force STOPPING state."""
        from competition_driver import DriveState, StopReason
        
        self.driver.state = DriveState.TRACKING
        self.driver.v2x_stop_zone = True
        
        mock_now = MagicMock()
        mock_now.to_sec.return_value = 100.0
        with patch('rospy.Time.now', return_value=mock_now):
            self.driver.last_lidar_time = mock_now
            self.driver.last_odom_time = mock_now
            self.driver.check_watchdog()
        
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        self.assertEqual(self.driver.stop_reason, StopReason.V2X_STOP_ZONE)
    
    def test_single_noise_cone_does_not_reset_timer(self):
        """Single noise cone should NOT refresh last_valid_time to prevent false keepalive."""
        from competition_driver import DriveState
        
        self.driver.state = DriveState.DEGRADED
        
        mock_now = MagicMock()
        initial_time = MagicMock()
        initial_time.to_sec.return_value = 50.0
        mock_now.to_sec.return_value = 100.0
        
        self.driver.last_valid_time = initial_time
        
        # Only 1 cone on one side (noise scenario)
        left_cones = [{'x': 5.0, 'y': 2.0, 'points': 5}]
        right_cones = []
        
        with patch('rospy.Time.now', return_value=mock_now):
            self.driver.update_state(left_cones, right_cones)
        
        # Timer should NOT have been updated (still old time)
        self.assertEqual(self.driver.last_valid_time, initial_time)
    
    def test_strong_perception_resets_timer(self):
        """Both-side cones (strong perception) should refresh last_valid_time."""
        from competition_driver import DriveState
        
        self.driver.state = DriveState.DEGRADED
        
        mock_now = MagicMock()
        initial_time = MagicMock()
        initial_time.to_sec.return_value = 50.0
        mock_now.to_sec.return_value = 100.0
        
        self.driver.last_valid_time = initial_time
        
        # Both sides have cones (strong perception)
        left_cones = [{'x': 5.0, 'y': 2.0, 'points': 5}]
        right_cones = [{'x': 5.0, 'y': -2.0, 'points': 5}]
        
        with patch('rospy.Time.now', return_value=mock_now):
            self.driver.update_state(left_cones, right_cones)
        
        # Timer should be updated to current time
        self.assertEqual(self.driver.last_valid_time, mock_now)
        self.assertEqual(self.driver.state, DriveState.TRACKING)

    def test_safety_stop_reasons_not_overridden_by_update_state(self):
        """Integration test: update_state() should NOT override safety STOPPING reasons."""
        from competition_driver import DriveState, StopReason
        
        # Setup: V2X stop zone active -> STOPPING state
        self.driver.state = DriveState.STOPPING
        self.driver.stop_reason = StopReason.V2X_STOP_ZONE
        
        mock_now = MagicMock()
        mock_now.to_sec.return_value = 100.0
        
        # Provide valid cones - this would normally promote to TRACKING
        left_cones = [{'x': 5.0, 'y': 2.0, 'points': 5}]
        right_cones = [{'x': 5.0, 'y': -2.0, 'points': 5}]
        
        with patch('rospy.Time.now', return_value=mock_now):
            self.driver.update_state(left_cones, right_cones)
        
        # State should REMAIN STOPPING - safety stop reasons are authoritative
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        self.assertEqual(self.driver.stop_reason, StopReason.V2X_STOP_ZONE)

    def test_estop_latched_requires_restart(self):
        """Integration test: E-stop should be latched and not auto-recover."""
        from competition_driver import DriveState, StopReason
        
        self.driver.state = DriveState.TRACKING
        self.driver.e_stop = True
        self.driver.e_stop_latched = True
        
        mock_now = MagicMock()
        mock_now.to_sec.return_value = 100.0
        
        with patch('rospy.Time.now', return_value=mock_now):
            self.driver.last_lidar_time = mock_now
            self.driver.last_odom_time = mock_now
            self.driver.check_watchdog()
        
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        self.assertEqual(self.driver.stop_reason, StopReason.EMERGENCY_STOP)
        
        # Even if e_stop signal goes false, latch keeps it STOPPING
        self.driver.e_stop = False
        
        with patch('rospy.Time.now', return_value=mock_now):
            self.driver.check_watchdog()
        
        # Should STILL be STOPPING due to latch
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        self.assertEqual(self.driver.stop_reason, StopReason.EMERGENCY_STOP)


if __name__ == '__main__':
    unittest.main()
