# FSDS AUTONOMOUS DRIVING - PROJECT KNOWLEDGE BASE

**Generated:** 2026-01-21
**Commit:** cb98a6d
**Branch:** master

## OVERVIEW

Docker-based ROS Noetic environment for Formula Student Driverless Simulator (FSDS) autonomous driving competition. Python scripts control vehicle via ROS topics, connecting to Windows-hosted FSDS simulator.

## STRUCTURE

```
hycu_fsds/
├── AGENTS.md                # This file
├── .gitlab-ci.yml           # CI/CD pipeline
├── fsds_docker/             # Main development directory
│   ├── Dockerfile           # ROS Noetic + FSDS (pinned deps)
│   ├── docker-compose.yml   # 7 services + healthchecks
│   ├── .env                 # FSDS_HOST_IP (Windows IP)
│   ├── start.sh             # One-click launcher + IP validation
│   ├── preflight_check.sh   # Demo rehearsal script
│   ├── dashboard.html       # Web UI dashboard
│   ├── entrypoint.sh        # Container init + ROS sourcing
│   ├── README.md            # Mermaid architecture + Safety section
│   ├── tests/
│   │   └── test_algorithms.py  # 22 unit tests (pytest/unittest)
│   └── scripts/
│       ├── competition_driver.py  # 750L - Pure Pursuit + V2X (MAIN)
│       ├── simple_slam.py         # 262L - Occupancy grid + TF tree
│       ├── v2x_rsu.py             # 336L - Virtual RSU (6 scenarios)
│       ├── lap_timer.py           # 191L - Performance + HUD
│       ├── cone_classifier.py     # 205L - Color classification + RViz
│       ├── basic_driver.py        #  83L - LiDAR obstacle stop
│       ├── autonomous_driver.py   # 107L - Cone detection + steering
│       └── advanced_driver.py     # 146L - PID + centerline
│
└── submission/              # Mirror of fsds_docker/ for packaging
    └── (same structure)     # Keep synced: rsync -av fsds_docker/ submission/
```

**Total Python LOC:** 2,372 lines

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Add new driver | `fsds_docker/scripts/` | Copy `competition_driver.py` as template |
| Change Windows IP | `fsds_docker/.env` | `FSDS_HOST_IP=your.ip` |
| Add Python deps | `fsds_docker/Dockerfile` | Line 52-58, `pip3 install` |
| Add ROS packages | `fsds_docker/Dockerfile` | Line 32-38, `apt-get` |
| Tune driving params | `scripts/*_driver.py` | `__init__` method or `rosparam set` |
| Run tests | Inside container | `python3 -m pytest tests/ -v` |
| Sync submission | Project root | `rsync -av fsds_docker/ submission/` |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `CompetitionDriver` | class | competition_driver.py | Pure Pursuit + V2X + Watchdog (MAIN) |
| `SimpleSLAM` | class | simple_slam.py | Occupancy grid + TF tree + decay |
| `V2X_RSU` | class | v2x_rsu.py | Virtual RSU (speed/hazard/stop) |
| `LapTimer` | class | lap_timer.py | Lap counting + HUD telemetry |
| `ConeClassifier` | class | cone_classifier.py | Intensity-based color classification |
| `DriveState` | enum | competition_driver.py | TRACKING/DEGRADED/STOPPING |
| `StopReason` | enum | competition_driver.py | Watchdog stop reasons |
| `find_cones_filtered` | method | competition_driver.py | Grid-based O(N) clustering |
| `pure_pursuit_steering` | method | competition_driver.py | Geometric path following |
| `estimate_curvature` | method | competition_driver.py | Gradient-based curvature |
| `calculate_target_speed` | method | competition_driver.py | √(a_lat_max / κ) model |
| `apply_rate_limit` | method | competition_driver.py | Smooth throttle/steering |

## KEY PARAMETERS

| Param | Default | ROS Param | Effect |
|-------|---------|-----------|--------|
| `max_throttle` | 0.25 | `~max_throttle` | Max acceleration |
| `min_speed` / `max_speed` | 2.0 / 6.0 m/s | `~min_speed`, `~max_speed` | Speed range |
| `max_steering` | 0.4 | `~max_steering` | Turn sharpness |
| `lookahead_base` | 4.0m | `~lookahead_base` | Pure Pursuit lookahead |
| `lidar_stale_timeout` | 1.0s | `~lidar_stale_timeout` | Watchdog threshold |
| `recovery_timeout` | 1.0s | `~recovery_timeout` | STOPPING → DEGRADED |
| `max_lateral_accel` | 4.0 m/s² | `~max_lateral_accel` | Curvature speed limit |
| `throttle_rate_limit` | 0.1 | `~throttle_rate_limit` | Smooth acceleration |
| `steering_rate_limit` | 0.15 | `~steering_rate_limit` | Smooth steering |

## ROS TOPICS

| Topic | Type | Direction |
|-------|------|-----------|
| `/fsds/control_command` | ControlCommand | Pub (throttle, steering, brake) |
| `/fsds/lidar/Lidar1` | PointCloud2 | Sub |
| `/fsds/testing_only/odom` | Odometry | Sub |
| `/slam/map` | OccupancyGrid | Pub |
| `/slam/path` | Path | Pub |
| `/slam/pose` | PoseStamped | Pub |
| `/v2x/speed_limit` | Float32 | Pub/Sub |
| `/v2x/hazard` | Bool | Pub/Sub |
| `/v2x/stop_zone` | Bool | Pub/Sub |
| `/v2x/rsu_status` | String (JSON) | Pub |
| `/lap/*` | std_msgs | Pub (count, time, speed) |
| `/debug/*` | std_msgs | Pub (8 telemetry topics) |
| `/cones/markers` | MarkerArray | Pub (RViz) |

## CONVENTIONS

- **No requirements.txt**: Python deps in Dockerfile only
- **No ROS package.xml on host**: Scripts mounted, packages built in container
- **Windows IP required**: Set `FSDS_HOST_IP` in `.env` before running
- **Thread safety**: `threading.Lock` for callback-main loop data
- **Exception safety**: `try/except Exception:` in all ROS callbacks
- **Data validation**: `np.isfinite()` guards on odom, V2X, control values
- **ROS params**: All tunable values exposed via `~param_name`
- **Academic docstrings**: All main files have 70+ line module docstrings with references

## ANTI-PATTERNS

| Forbidden | Why |
|-----------|-----|
| `pip install` in container | Add to Dockerfile for reproducibility |
| Hardcoded IPs | Use `.env` file |
| Missing `rospy.init_node` | Every driver must call in `__init__` |
| Repeated `rospy.get_param` | Cache params in `__init__` for hot paths |
| Custom ROS msg types | Use `std_msgs` to avoid build issues |
| Complex SLAM (scan matching, loop closure) | Stick to simple occupancy grid |
| `queue_size > 1` for control | Real-time control needs `queue_size=1` |
| `as any`, `@ts-ignore` | Never suppress type errors |
| Bare `except:` | Always `except Exception:` |
| O(N²) clustering | Use grid-based O(N) in hot paths |

## COMMANDS

```bash
# Start environment
./fsds_docker/start.sh

# Enter dev container
docker exec -it fsds_dev bash

# Run driver (inside container)
python3 /root/catkin_ws/src/fsds_scripts/scripts/competition_driver.py

# Run SLAM (separate terminal)
python3 /root/catkin_ws/src/fsds_scripts/scripts/simple_slam.py

# Run V2X RSU
python3 /root/catkin_ws/src/fsds_scripts/scripts/v2x_rsu.py

# Run tests
python3 -m pytest tests/ -v

# View topics
rostopic list
rostopic echo /fsds/testing_only/odom

# Tune params at runtime
rosparam set /competition_driver/max_speed 8.0

# Sync submission directory
rsync -av --exclude='__pycache__' fsds_docker/ submission/

# Visualize
docker-compose --profile viz up -d rviz

# Stop all
docker-compose down
```

## TESTING

| Test Suite | Count | Coverage |
|------------|-------|----------|
| Pure Pursuit steering | 6 | Geometric calculations |
| Curvature estimation | 4 | Path analysis |
| Lookahead selection | 3 | Arc-length vs Euclidean |
| Cone filtering | 2 | Z-height, clustering |
| Speed control | 3 | Curvature-based limits |
| Watchdog state machine | 4 | CONES_LOST, V2X stop, perception |
| **Total** | **22** | Core algorithms |

Run: `python3 -m pytest tests/ -v` (inside container)

## REFERENCES (in docstrings)

| Topic | Reference |
|-------|-----------|
| Pure Pursuit | Coulter, R.C. (1992). CMU-RI-TR-92-01 |
| Occupancy Grid | Thrun, S. (2005). Probabilistic Robotics |
| Grid-based SLAM | Grisetti, G. et al. (2007). GMapping |
| V2X Standards | SAE J2735, ETSI TS 103 301 |

## NOTES

- Windows FSDS simulator must be running BEFORE `start.sh`
- ROS Bridge connects via port 41451
- Container scripts auto-mounted from `./scripts/` - edit on host, run in container
- `competition_driver.py` is the main submission for the competition
- `submission/` directory is a duplicate of `fsds_docker/` for packaging - keep synced
- Service startup order: roscore → fsds_bridge → [dev, lap_timer, cone_classifier]
- All drivers follow 20Hz control loop pattern
- Academic docstrings include algorithm explanations for code review scoring
