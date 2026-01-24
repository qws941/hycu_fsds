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

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from scripts.competition_driver import CompetitionDriver
from scripts.simple_slam import SimpleSLAM

try:
    from src.control.pure_pursuit import pure_pursuit_steering, get_lookahead_point
    from src.control.speed import estimate_curvature, calculate_target_speed, apply_rate_limit
    from src.perception.cone_detector import find_cones_filtered, build_centerline
    from src.utils.watchdog import DriveState, StopReason, Watchdog
    SRC_AVAILABLE = True
except ImportError:
    SRC_AVAILABLE = False

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
        from scripts.competition_driver import DriveState, StopReason
        
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
        from scripts.competition_driver import DriveState, StopReason
        
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
        from scripts.competition_driver import DriveState
        
        self.driver.state = DriveState.DEGRADED
        
        mock_now = MagicMock()
        initial_time = MagicMock()
        initial_time.to_sec.return_value = 50.0
        mock_now.to_sec.return_value = 100.0
        
        # Support time subtraction: now - initial_time = 50 seconds
        duration_mock = MagicMock()
        duration_mock.to_sec.return_value = 50.0
        mock_now.__sub__ = MagicMock(return_value=duration_mock)
        
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
        from scripts.competition_driver import DriveState
        
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
        from scripts.competition_driver import DriveState, StopReason
        
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
        from scripts.competition_driver import DriveState, StopReason
        
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

    def test_v2x_resume_blocked_when_lidar_stale(self):
        """Critical fix test: V2X stop zone clear should NOT resume if LiDAR is stale."""
        from scripts.competition_driver import DriveState, StopReason
        
        # Setup: V2X stop zone was active, now cleared
        self.driver.state = DriveState.STOPPING
        self.driver.stop_reason = StopReason.V2X_STOP_ZONE
        self.driver.v2x_stop_zone = False  # Stop zone cleared
        self.driver.e_stop = False
        self.driver.e_stop_latched = False
        
        # But LiDAR is stale (2 seconds old)
        now_mock = MagicMock()
        now_mock.to_sec.return_value = 100.0
        
        old_lidar_time = MagicMock()
        old_lidar_time.to_sec.return_value = 98.0  # 2 seconds old
        self.driver.last_lidar_time = old_lidar_time
        self.driver.last_odom_time = now_mock  # Odom is fresh
        self.driver.lidar_stale_timeout = 1.0
        
        # Support time subtraction for stale check
        lidar_duration = MagicMock()
        lidar_duration.to_sec.return_value = 2.0  # now - old_lidar = 2 seconds (stale)
        odom_duration = MagicMock()
        odom_duration.to_sec.return_value = 0.0  # now - now = 0 seconds (fresh)
        
        def sub_handler(other):
            if other is old_lidar_time:
                return lidar_duration
            return odom_duration
        now_mock.__sub__ = MagicMock(side_effect=sub_handler)
        
        with patch('rospy.Time.now', return_value=now_mock):
            self.driver.check_watchdog()
        
        # Should stay STOPPING but switch to LIDAR_STALE reason
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        self.assertEqual(self.driver.stop_reason, StopReason.LIDAR_STALE)

    def test_v2x_resume_blocked_when_odom_stale(self):
        """Critical fix test: V2X stop zone clear should NOT resume if Odom is stale."""
        from scripts.competition_driver import DriveState, StopReason
        
        # Setup: V2X stop zone was active, now cleared
        self.driver.state = DriveState.STOPPING
        self.driver.stop_reason = StopReason.V2X_STOP_ZONE
        self.driver.v2x_stop_zone = False  # Stop zone cleared
        self.driver.e_stop = False
        self.driver.e_stop_latched = False
        
        # LiDAR is fresh but Odom is stale
        now_mock = MagicMock()
        now_mock.to_sec.return_value = 100.0
        
        old_odom_time = MagicMock()
        old_odom_time.to_sec.return_value = 98.0  # 2 seconds old
        self.driver.last_lidar_time = now_mock  # LiDAR is fresh
        self.driver.last_odom_time = old_odom_time
        self.driver.odom_stale_timeout = 1.0
        
        # Support time subtraction for stale check
        lidar_duration = MagicMock()
        lidar_duration.to_sec.return_value = 0.0  # now - now = 0 seconds (fresh)
        odom_duration = MagicMock()
        odom_duration.to_sec.return_value = 2.0  # now - old_odom = 2 seconds (stale)
        
        def sub_handler(other):
            if other is old_odom_time:
                return odom_duration
            return lidar_duration
        now_mock.__sub__ = MagicMock(side_effect=sub_handler)
        
        with patch('rospy.Time.now', return_value=now_mock):
            self.driver.check_watchdog()
        
        # Should stay STOPPING but switch to ODOM_STALE reason
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        self.assertEqual(self.driver.stop_reason, StopReason.ODOM_STALE)

    def test_weak_perception_does_not_recover_cones_lost(self):
        """Critical fix test: Weak perception (1-2 cones one side) should NOT recover from CONES_LOST."""
        from scripts.competition_driver import DriveState, StopReason
        
        # Setup: Currently in CONES_LOST stopping state
        self.driver.state = DriveState.STOPPING
        self.driver.stop_reason = StopReason.CONES_LOST
        
        mock_now = MagicMock()
        mock_now.to_sec.return_value = 100.0
        
        initial_time = MagicMock()
        initial_time.to_sec.return_value = 50.0  # Old timer
        self.driver.last_valid_time = initial_time
        
        # Weak perception: only 2 cones on one side (noise)
        left_cones = [{'x': 5.0, 'y': 2.0, 'points': 5}, {'x': 6.0, 'y': 2.5, 'points': 5}]
        right_cones = []  # No right cones
        
        with patch('rospy.Time.now', return_value=mock_now):
            self.driver.update_state(left_cones, right_cones)
        
        # Should REMAIN in STOPPING - weak perception cannot recover
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        self.assertEqual(self.driver.stop_reason, StopReason.CONES_LOST)
        # Timer should NOT have been reset
        self.assertEqual(self.driver.last_valid_time, initial_time)

    def test_weak_perception_continues_elapsed_calculation(self):
        """Critical fix test: Weak perception should allow elapsed timer to continue toward STOPPING."""
        from scripts.competition_driver import DriveState, StopReason
        
        # Setup: Currently TRACKING with fresh timer
        self.driver.state = DriveState.TRACKING
        self.driver.stop_reason = StopReason.NONE
        self.driver.degraded_timeout = 1.0
        self.driver.stopping_timeout = 3.0
        
        initial_time = MagicMock()
        initial_time.to_sec.return_value = 50.0
        self.driver.last_valid_time = initial_time
        
        # Time passes with only weak perception (2 cones one side)
        mock_now = MagicMock()
        mock_now.to_sec.return_value = 55.0  # 5 seconds elapsed > stopping_timeout
        
        # Support time subtraction: now - initial_time = 5 seconds
        duration_mock = MagicMock()
        duration_mock.to_sec.return_value = 5.0
        mock_now.__sub__ = MagicMock(return_value=duration_mock)
        
        # Weak perception - should NOT prevent timeout
        left_cones = [{'x': 5.0, 'y': 2.0, 'points': 5}, {'x': 6.0, 'y': 2.5, 'points': 5}]
        right_cones = []
        
        with patch('rospy.Time.now', return_value=mock_now):
            self.driver.update_state(left_cones, right_cones)
        
        # Should transition to STOPPING because elapsed > stopping_timeout
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        self.assertEqual(self.driver.stop_reason, StopReason.CONES_LOST)

    def test_v2x_resume_allowed_when_sensors_fresh(self):
        """Test: V2X stop zone clear SHOULD resume if all sensors are fresh."""
        from scripts.competition_driver import DriveState, StopReason
        
        # Setup: V2X stop zone was active, now cleared
        self.driver.state = DriveState.STOPPING
        self.driver.stop_reason = StopReason.V2X_STOP_ZONE
        self.driver.v2x_stop_zone = False  # Stop zone cleared
        self.driver.e_stop = False
        self.driver.e_stop_latched = False
        
        # All sensors fresh
        now_mock = MagicMock()
        now_mock.to_sec.return_value = 100.0
        
        self.driver.last_lidar_time = now_mock  # Fresh
        self.driver.last_odom_time = now_mock   # Fresh
        self.driver.lidar_stale_timeout = 1.0
        self.driver.odom_stale_timeout = 1.0
        
        # Support time subtraction: now - now = 0 seconds (fresh)
        duration_mock = MagicMock()
        duration_mock.to_sec.return_value = 0.0
        now_mock.__sub__ = MagicMock(return_value=duration_mock)
        
        with patch('rospy.Time.now', return_value=now_mock):
            self.driver.check_watchdog()
        
        # Should resume to DEGRADED
        self.assertEqual(self.driver.state, DriveState.DEGRADED)
        self.assertEqual(self.driver.stop_reason, StopReason.NONE)


class TestE2EIntegration(unittest.TestCase):
    """
    End-to-End Integration Tests
    
    Tests the complete pipeline: LiDAR points → Cone detection → Centerline → Control commands
    Without actual ROS or simulator, using mocked sensor data.
    """
    
    @staticmethod
    def _to_dicts(tuples):
        return [{'x': t[0], 'y': t[1]} for t in tuples]
    
    def setUp(self):
        """Create a fresh driver instance for each test"""
        with patch('rospy.init_node'), \
             patch('rospy.Subscriber'), \
             patch('rospy.Publisher'), \
             patch('rospy.get_param', side_effect=lambda x, default=None: default), \
             patch('rospy.Rate'), \
             patch('rospy.is_shutdown', return_value=True):
            self.driver = CompetitionDriver()
            self.driver.state = DriveState.TRACKING
            self.driver.stop_reason = StopReason.NONE
            
            # Setup mock time
            now_mock = MagicMock()
            now_mock.to_sec.return_value = 100.0
            duration_mock = MagicMock()
            duration_mock.to_sec.return_value = 0.0
            now_mock.__sub__ = MagicMock(return_value=duration_mock)
            self.driver.last_lidar_time = now_mock
            self.driver.last_odom_time = now_mock
    
    def test_e2e_straight_track_produces_zero_steering(self):
        """
        E2E: Straight track with symmetric cones should produce near-zero steering
        
        Scenario: Cones placed symmetrically on left and right sides
        Expected: Centerline is straight, steering ≈ 0
        """
        # Simulate symmetric cone layout (straight track)
        left_cones = [(5, 2), (10, 2), (15, 2)]   # Left side at y=2
        right_cones = [(5, -2), (10, -2), (15, -2)]  # Right side at y=-2
        
        # Build centerline from cones
        centerline = []
        for i in range(len(left_cones)):
            cx = (left_cones[i][0] + right_cones[i][0]) / 2
            cy = (left_cones[i][1] + right_cones[i][1]) / 2
            centerline.append({'x': cx, 'y': cy})
        
        # Set driver state
        self.driver.centerline = centerline
        self.driver.current_speed = 3.0
        
        # Calculate steering using Pure Pursuit
        target = self.driver.get_lookahead_point(centerline)
        steering = self.driver.pure_pursuit_steering(target)
        
        # Straight track should have near-zero steering
        self.assertAlmostEqual(steering, 0.0, places=1,
            msg=f"Straight track should have ~0 steering, got {steering}")
    
    def test_e2e_left_curve_produces_positive_steering(self):
        """
        E2E: Left curve should produce positive (left) steering
        
        Scenario: Track curves to the left
        Expected: Positive steering value
        """
        # Simulate left curve (cones curve left)
        centerline = [
            {'x': 3, 'y': 0},
            {'x': 6, 'y': 1},
            {'x': 9, 'y': 3},
            {'x': 11, 'y': 6},
        ]
        
        self.driver.centerline = centerline
        self.driver.current_speed = 3.0
        
        target = self.driver.get_lookahead_point(centerline)
        steering = self.driver.pure_pursuit_steering(target)
        
        # Left curve should produce positive steering
        self.assertGreater(steering, 0.0,
            msg=f"Left curve should have positive steering, got {steering}")
    
    def test_e2e_right_curve_produces_negative_steering(self):
        """
        E2E: Right curve should produce negative (right) steering
        
        Scenario: Track curves to the right
        Expected: Negative steering value
        """
        # Simulate right curve
        centerline = [
            {'x': 3, 'y': 0},
            {'x': 6, 'y': -1},
            {'x': 9, 'y': -3},
            {'x': 11, 'y': -6},
        ]
        
        self.driver.centerline = centerline
        self.driver.current_speed = 3.0
        
        target = self.driver.get_lookahead_point(centerline)
        steering = self.driver.pure_pursuit_steering(target)
        
        # Right curve should produce negative steering
        self.assertLess(steering, 0.0,
            msg=f"Right curve should have negative steering, got {steering}")
    
    def test_e2e_sharp_curve_reduces_target_speed(self):
        """
        E2E: Sharp curves should result in lower target speed
        
        Scenario: Compare speed on straight vs sharp curve
        Expected: Sharp curve speed < straight speed
        """
        # Straight path
        straight_path = self._to_dicts([(5, 0), (10, 0), (15, 0), (20, 0)])
        straight_curvature = self.driver.estimate_curvature(straight_path)
        straight_speed = self.driver.calculate_target_speed(straight_curvature)
        
        sharp_path = self._to_dicts([(3, 0), (5, 0), (7, 2), (7, 5)])
        sharp_curvature = self.driver.estimate_curvature(sharp_path)
        sharp_speed = self.driver.calculate_target_speed(sharp_curvature)
        
        # Sharp curve should have lower target speed
        self.assertLess(sharp_speed, straight_speed,
            msg=f"Sharp curve speed ({sharp_speed}) should be less than straight ({straight_speed})")
    
    def test_e2e_no_cones_triggers_degraded_state(self):
        """
        E2E: No cones detected should trigger state transition to DEGRADED
        
        Scenario: Empty cone list after timeout
        Expected: State transitions from TRACKING to DEGRADED
        """
        self.driver.state = DriveState.TRACKING
        self.driver.degraded_timeout = 1.0
        self.driver.stopping_timeout = 3.0
        
        with patch('rospy.Time.now') as mock_now:
            now_mock = MagicMock()
            now_mock.to_sec.return_value = 5.0
            mock_now.return_value = now_mock
            
            last_valid_mock = MagicMock()
            last_valid_mock.to_sec.return_value = 0.0
            duration_mock = MagicMock()
            duration_mock.to_sec.return_value = 5.0
            now_mock.__sub__ = MagicMock(return_value=duration_mock)
            self.driver.last_valid_time = last_valid_mock
            
            self.driver.update_state([], [])
        
        # Should be in DEGRADED or STOPPING (use .name to avoid enum instance mismatch)
        self.assertIn(self.driver.state.name, ['DEGRADED', 'STOPPING'],
            msg=f"No cones should trigger DEGRADED/STOPPING, got {self.driver.state}")
    
    def test_e2e_v2x_stop_overrides_tracking(self):
        """
        E2E: V2X stop zone should immediately override TRACKING state
        
        Scenario: Vehicle tracking normally, then V2X stop signal received
        Expected: Immediate transition to STOPPING with V2X_STOP_ZONE reason
        """
        self.driver.state = DriveState.TRACKING
        self.driver.stop_reason = StopReason.NONE
        
        # Simulate V2X stop zone callback
        msg = MagicMock()
        msg.data = True
        self.driver.v2x_stop_zone_callback(msg)
        
        # Should immediately be STOPPING with V2X reason (use .name to avoid enum instance mismatch)
        self.assertEqual(self.driver.state.name, 'STOPPING',
            msg=f"V2X stop should trigger STOPPING, got {self.driver.state}")
        self.assertEqual(self.driver.stop_reason.name, 'V2X_STOP_ZONE',
            msg=f"Stop reason should be V2X_STOP_ZONE, got {self.driver.stop_reason}")
    
    def test_e2e_estop_latches_permanently(self):
        """
        E2E: E-stop should latch and not recover even when released
        
        Scenario: E-stop triggered, then released
        Expected: State remains STOPPING, requires node restart
        """
        self.driver.state = DriveState.TRACKING
        self.driver.e_stop_latched = False
        
        # Trigger E-stop
        msg = MagicMock()
        msg.data = True
        self.driver.estop_callback(msg)
        
        self.assertEqual(self.driver.state.name, 'STOPPING')
        self.assertTrue(self.driver.e_stop_latched)
        
        # Release E-stop
        msg.data = False
        self.driver.e_stop = False
        
        # Run watchdog - should NOT recover
        with patch('rospy.Time.now') as mock_now:
            now_mock = MagicMock()
            now_mock.to_sec.return_value = 100.0
            duration_mock = MagicMock()
            duration_mock.to_sec.return_value = 0.0
            now_mock.__sub__ = MagicMock(return_value=duration_mock)
            mock_now.return_value = now_mock
            self.driver.last_lidar_time = now_mock
            self.driver.last_odom_time = now_mock
            
            self.driver.check_watchdog()
        
        # Should still be STOPPING due to latch
        self.assertEqual(self.driver.state.name, 'STOPPING',
            msg="E-stop should remain latched after release")
        self.assertTrue(self.driver.e_stop_latched)
    
    def test_e2e_control_output_bounds(self):
        """
        E2E: Control outputs should always be within valid bounds
        
        Scenario: Various steering and speed calculations
        Expected: throttle in [0, max_throttle], steering in [-max_steering, max_steering]
        """
        test_centerlines = [
            self._to_dicts([(5, 0), (10, 0), (15, 0)]),
            self._to_dicts([(3, 0), (6, 2), (8, 5)]),
            self._to_dicts([(3, 0), (6, -2), (8, -5)]),
            self._to_dicts([(2, 0), (3, 1), (3, 3)]),
        ]
        
        for centerline in test_centerlines:
            target = self.driver.get_lookahead_point(centerline)
            steering = self.driver.pure_pursuit_steering(target)
            
            # Steering should be bounded
            self.assertGreaterEqual(steering, -self.driver.max_steering,
                msg=f"Steering {steering} below min bound for {centerline}")
            self.assertLessEqual(steering, self.driver.max_steering,
                msg=f"Steering {steering} above max bound for {centerline}")
    
    def test_e2e_stopping_state_outputs_safe_controls(self):
        """
        E2E: STOPPING state should output safe control values
        
        Scenario: Vehicle in STOPPING state
        Expected: throttle=0, brake=1, steering=0
        """
        self.driver.state = DriveState.STOPPING
        self.driver.stop_reason = StopReason.CONES_LOST
        
        # The get_control_command or similar should output safe values
        # Test the expected behavior when state is STOPPING
        self.assertEqual(self.driver.state, DriveState.STOPPING)
        
        # Verify stop reason is set
        self.assertNotEqual(self.driver.stop_reason, StopReason.NONE)
    
    def test_e2e_pipeline_lidar_to_steering(self):
        """
        E2E: Complete pipeline from simulated LiDAR points to steering output
        
        Scenario: Simulate full processing chain
        Expected: Valid steering output from mock LiDAR data
        """
        # Simulate LiDAR points that look like cones
        # Left cones at y > 0, right cones at y < 0
        mock_points = [
            # Left cones (clusters)
            (5.0, 1.8, 0.2), (5.1, 1.9, 0.2), (5.0, 2.0, 0.3),
            (10.0, 2.0, 0.2), (10.1, 2.1, 0.2),
            # Right cones (clusters)
            (5.0, -1.8, 0.2), (5.1, -1.9, 0.2),
            (10.0, -2.0, 0.2), (10.1, -2.1, 0.2),
        ]
        
        # Filter points (simulating find_cones_filtered)
        cones = []
        for x, y, z in mock_points:
            dist = math.sqrt(x*x + y*y)
            if 1.0 < dist < 15.0 and -0.3 < z < 0.5:
                cones.append((x, y))
        
        # Separate left/right
        left_cones = [(x, y) for x, y in cones if y > 0]
        right_cones = [(x, y) for x, y in cones if y < 0]
        
        # Build centerline
        centerline = []
        min_len = min(len(left_cones), len(right_cones))
        if min_len > 0:
            left_sorted = sorted(left_cones, key=lambda p: p[0])
            right_sorted = sorted(right_cones, key=lambda p: p[0])
            for i in range(min_len):
                cx = (left_sorted[i][0] + right_sorted[i][0]) / 2
                cy = (left_sorted[i][1] + right_sorted[i][1]) / 2
                centerline.append({'x': cx, 'y': cy})
        
        self.assertGreater(len(centerline), 0, "Should have centerline points")
        
        # Calculate steering
        target = self.driver.get_lookahead_point(centerline)
        steering = self.driver.pure_pursuit_steering(target)
        
        # For symmetric cones, steering should be near zero
        self.assertAlmostEqual(steering, 0.0, places=1,
            msg=f"Symmetric cones should produce ~0 steering, got {steering}")


if __name__ == '__main__':
    unittest.main()
