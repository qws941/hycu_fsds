# FSDS AUTONOMOUS DRIVING - PROJECT KNOWLEDGE BASE

**Generated:** 2026-01-25
**Commit:** f0ae405
**Branch:** docs/update-agents

## OVERVIEW

Docker-based ROS Noetic environment for Formula Student Driverless Simulator (FSDS) autonomous driving competition. Python scripts control vehicle via ROS topics, connecting to Windows-hosted FSDS simulator.

## STRUCTURE

```
hycu_fsds/
├── AGENTS.md                # This file
├── .gitlab-ci.yml           # CI/CD pipeline
├── src/
│   ├── autonomous/          # Main development directory
│   │   ├── Dockerfile       # ROS Noetic + FSDS (pinned deps)
│   │   ├── docker-compose.yml  # 3 services: roscore, fsds_bridge, autonomous
│   │   ├── .env             # FSDS_HOST_IP (Windows IP)
│   │   ├── start.sh         # One-click launcher
│   │   ├── entrypoint.sh    # Container init + ROS sourcing
│   │   │
│   │   ├── driver/          # Competition driver
│   │   │   └── competition_driver.py  # 832L - Pure Pursuit + V2X + 3-tier perception
│   │   │
│   │   ├── modules/         # Modular components
│   │   │   ├── control/     # pure_pursuit.py (96L), speed.py (190L)
│   │   │   ├── perception/  # cone_detector.py (222L), slam.py (262L), cone_classifier.py (205L)
│   │   │   └── utils/       # watchdog.py (251L), lap_timer.py (191L)
│   │   │
│   │   ├── config/
│   │   │   └── params.yaml  # All ROS parameters
│   │   │
│   │   └── tests/
│   │       └── test_algorithms.py  # 846L - 29 unit tests
│   │
│   └── simulator/           # Windows FSDS settings.json
│
├── scripts/
│   └── package.sh           # Distribution packaging
│
├── docs/
│   └── reference_materials/ # Lecture notes (Korean)
│
├── dist/                    # Built packages
│
└── fsds_docker/             # DEPRECATED - legacy structure
```

**Total Python LOC:** ~3,141 lines

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Main driver logic | `src/autonomous/driver/competition_driver.py` | 832 lines, monolithic |
| Control algorithms | `src/autonomous/modules/control/` | Pure functions, no ROS deps |
| Perception modules | `src/autonomous/modules/perception/` | Cone detection, SLAM |
| Tune driving params | `src/autonomous/config/params.yaml` | YAML config |
| Change Windows IP | `src/autonomous/.env` | `FSDS_HOST_IP=your.ip` |
| Add Python deps | `src/autonomous/Dockerfile` | `pip3 install` section |
| Run tests | Inside container | `python3 -m pytest tests/ -v` |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `CompetitionDriver` | class | driver/competition_driver.py | Main driver (monolithic) |
| `pure_pursuit_steering` | func | modules/control/pure_pursuit.py | Geometric steering |
| `get_lookahead_point` | func | modules/control/pure_pursuit.py | Find target on path |
| `estimate_curvature` | func | modules/control/speed.py | Gradient-based curvature |
| `calculate_target_speed` | func | modules/control/speed.py | v = sqrt(a_lat_max / k) |
| `apply_rate_limit` | func | modules/control/speed.py | Smooth control changes |
| `find_cones_filtered` | func | modules/perception/cone_detector.py | Grid BFS clustering O(N) |
| `build_centerline` | func | modules/perception/cone_detector.py | Left/right -> center path |
| `DriveState` | enum | modules/utils/watchdog.py | TRACKING/DEGRADED/STOPPING |
| `StopReason` | enum | modules/utils/watchdog.py | Watchdog stop reasons |
| `Watchdog` | class | modules/utils/watchdog.py | Sensor timeout monitoring |

## KEY PARAMETERS

All in `config/params.yaml`:

| Param | Default | Effect |
|-------|---------|--------|
| `max_throttle` | 0.25 | Max acceleration |
| `min_speed` / `max_speed` | 2.0 / 6.0 m/s | Speed range |
| `max_steering` | 0.4 rad | Turn sharpness |
| `lookahead_base` | 4.0m | Pure Pursuit lookahead |
| `lidar_stale_timeout` | 1.0s | Watchdog threshold |
| `cone_grouping_threshold` | 0.2m | Grid cell size |
| `cone_min_points` | 3 | Min points per cluster |

## ROS TOPICS

| Topic | Type | Direction |
|-------|------|-----------|
| `/fsds/control_command` | ControlCommand | Pub |
| `/fsds/lidar/Lidar1` | PointCloud2 | Sub |
| `/fsds/testing_only/odom` | Odometry | Sub |
| `/v2x/speed_limit` | Float32 | Sub |
| `/v2x/hazard` | Bool | Sub |
| `/v2x/stop_zone` | Bool | Sub |

## CONVENTIONS

- **No requirements.txt**: Python deps in Dockerfile only
- **Windows IP required**: Set `FSDS_HOST_IP` in `.env` before running
- **Thread safety**: `threading.Lock` for callback-main loop data
- **Exception safety**: `try/except Exception:` in all ROS callbacks
- **Data validation**: `np.isfinite()` guards on odom, V2X, control values
- **PYTHONPATH**: Must include `/root/catkin_ws/src/autonomous`

## ANTI-PATTERNS

| Forbidden | Why |
|-----------|-----|
| `pip install` in container | Add to Dockerfile for reproducibility |
| Hardcoded IPs | Use `.env` file |
| Missing `rospy.init_node` | Every driver must call in `__init__` |
| `queue_size > 1` for control | Real-time control needs `queue_size=1` |
| Bare `except:` | Always `except Exception:` |
| O(N^2) clustering | Use grid-based O(N) in hot paths |
| Edit `fsds_docker/` | DEPRECATED - use `src/autonomous/` |

## COMMANDS

```bash
# Start environment
cd src/autonomous && ./start.sh

# Enter dev container
docker exec -it fsds_autonomous bash

# Run driver (inside container)
python3 /root/catkin_ws/src/autonomous/driver/competition_driver.py

# Run tests
python3 -m pytest tests/ -v

# View topics
rostopic list
rostopic echo /fsds/testing_only/odom

# Tune params at runtime
rosparam set /competition_driver/max_speed 8.0

# Stop all
docker-compose down
```

## TESTING

| Test Class | Count | Coverage |
|------------|-------|----------|
| TestCompetitionDriver | 6 | Pure Pursuit, curvature, speed |
| TestSimpleSLAM | 2 | Grid conversion |
| TestWatchdogStateMachine | 11 | E-stop, V2X, perception tiers |
| TestE2EIntegration | 8 | End-to-end scenarios |
| **Total** | **27** | Core algorithms |

Run: `python3 -m pytest tests/ -v` (inside container)

## ALGORITHMS

1. **Cone Detection**: Grid BFS clustering O(N), Z-height filter (-0.3m ~ 0.5m)
2. **Steering**: Pure Pursuit `delta = atan2(2 * wheelbase * sin(alpha) / L)`
3. **Speed**: Curvature-based `v = sqrt(a_lat_max / kappa)`
4. **State Machine**: TRACKING -> DEGRADED (1s) -> STOPPING (3s)
5. **3-tier Perception**: strong (both sides OR 3+) / weak (1-2 one side) / none

## NOTES

- Windows FSDS simulator must be running BEFORE `start.sh`
- ROS Bridge connects via port 41451
- Container scripts auto-mounted - edit on host, run in container
- `fsds_docker/` is DEPRECATED - all development in `src/autonomous/`
- All drivers follow 20Hz control loop pattern
