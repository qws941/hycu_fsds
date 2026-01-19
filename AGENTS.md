# FSDS AUTONOMOUS DRIVING - PROJECT KNOWLEDGE BASE

**Generated:** 2026-01-19
**Commit:** 31e4045
**Branch:** master

## OVERVIEW

Docker-based ROS Noetic environment for Formula Student Driverless Simulator (FSDS) autonomous driving competition. Python scripts control vehicle via ROS topics, connecting to Windows-hosted FSDS simulator.

## STRUCTURE

```
fsds_docker/
├── Dockerfile           # ROS Noetic + FSDS (pinned deps)
├── docker-compose.yml   # 7 services + healthchecks
├── .env                 # FSDS_HOST_IP (Windows IP)
├── start.sh             # One-click launcher + IP validation
├── preflight_check.sh   # Demo rehearsal script
├── dashboard.html       # Web UI dashboard
├── entrypoint.sh        # Container init + ROS sourcing
├── tests/
│   └── test_algorithms.py  # Unit tests (pytest/unittest)
├── rviz/                # RViz configurations
└── scripts/
    ├── competition_driver.py  # Pure Pursuit + V2X (MAIN)
    ├── simple_slam.py         # Occupancy grid + TF tree
    ├── v2x_rsu.py             # Virtual RSU (6 scenarios)
    ├── lap_timer.py           # Performance + HUD
    ├── cone_classifier.py     # Color classification + RViz
    ├── basic_driver.py        # LiDAR obstacle stop
    ├── autonomous_driver.py   # Cone detection + steering
    └── advanced_driver.py     # PID + centerline

submission/                    # IGNORE: duplicate for packaging
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Add new driver | `fsds_docker/scripts/` | Copy `competition_driver.py` as template |
| Change Windows IP | `fsds_docker/.env` | `FSDS_HOST_IP=your.ip` |
| Add Python deps | `fsds_docker/Dockerfile` | Line 52-58, `pip3 install` |
| Add ROS packages | `fsds_docker/Dockerfile` | Line 32-38, `apt-get` |
| Tune driving params | `scripts/*_driver.py` | `__init__` method or `rosparam set` |
| Run tests | Inside container | `python3 -m pytest tests/ -v` |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `CompetitionDriver` | class | competition_driver.py | Pure Pursuit + V2X + Watchdog (MAIN) |
| `SimpleSLAM` | class | simple_slam.py | Occupancy grid + TF tree + decay |
| `V2X_RSU` | class | v2x_rsu.py | Virtual RSU (speed/hazard/stop) |
| `DriveState` | enum | competition_driver.py | TRACKING/DEGRADED/STOPPING |
| `find_cones_filtered` | method | competition_driver.py | Z-filter + cluster validation |
| `pure_pursuit_steering` | method | competition_driver.py | Geometric path following |
| `estimate_curvature` | method | competition_driver.py | Path curvature → speed control |

## KEY PARAMETERS

| Param | Default | ROS Param | Effect |
|-------|---------|-----------|--------|
| `max_throttle` | 0.25 | `~max_throttle` | Max acceleration |
| `min_speed` / `max_speed` | 2.0 / 6.0 m/s | `~min_speed`, `~max_speed` | Speed range |
| `max_steering` | 0.4 | `~max_steering` | Turn sharpness |
| `lookahead_base` | 4.0m | `~lookahead_base` | Pure Pursuit lookahead |
| `lidar_stale_timeout` | 1.0s | `~lidar_stale_timeout` | Watchdog threshold |
| `max_lateral_accel` | 4.0 m/s² | `~max_lateral_accel` | Curvature speed limit |

## ROS TOPICS

| Topic | Type | Direction |
|-------|------|-----------|
| `/fsds/control_command` | ControlCommand | Pub (throttle, steering, brake) |
| `/fsds/lidar/Lidar1` | PointCloud2 | Sub |
| `/fsds/testing_only/odom` | Odometry | Sub |
| `/slam/map` | OccupancyGrid | Pub |
| `/slam/path` | Path | Pub |
| `/v2x/speed_limit` | Float32 | Pub/Sub |
| `/v2x/hazard` | Bool | Pub/Sub |
| `/debug/*` | std_msgs | Pub (8 telemetry topics) |

## CONVENTIONS

- **No requirements.txt**: Python deps in Dockerfile only
- **No ROS package.xml on host**: Scripts mounted, packages built in container
- **Windows IP required**: Set `FSDS_HOST_IP` in `.env` before running
- **Thread safety**: `threading.Lock` for callback-main loop data
- **Exception safety**: Try/except in all ROS callbacks with throttled logging
- **Data validation**: `np.isfinite()` guards on odom, V2X, control values
- **ROS params**: All tunable values exposed via `~param_name`

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

# Run tests
python3 -m pytest tests/ -v

# View topics
rostopic list
rostopic echo /fsds/testing_only/odom

# Tune params at runtime
rosparam set /competition_driver/max_speed 8.0

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

Run: `python3 -m pytest tests/ -v` (inside container)

## NOTES

- Windows FSDS simulator must be running BEFORE `start.sh`
- ROS Bridge connects via port 41451
- Container scripts auto-mounted from `./scripts/` - edit on host, run in container
- `competition_driver.py` is the main submission for the competition
- `submission/` directory is a duplicate of `fsds_docker/` for packaging - ignore for development
- Service startup order: roscore → fsds_bridge → [dev, lap_timer, cone_classifier]
- All drivers follow 20Hz control loop pattern
