# FSDS AUTONOMOUS DRIVING - PROJECT KNOWLEDGE BASE

**Generated:** 2026-01-21
**Commit:** 1d661b9
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
│   │
│   ├── scripts/             # Original monolithic scripts (deprecated backup)
│   │   ├── competition_driver.py  # 784L - Original Pure Pursuit + V2X
│   │   ├── simple_slam.py         # 262L - Occupancy grid + TF tree
│   │   ├── v2x_rsu.py             # 336L - Virtual RSU (6 scenarios)
│   │   ├── lap_timer.py           # 191L - Performance + HUD
│   │   ├── cone_classifier.py     # 205L - Color classification + RViz
│   │   ├── basic_driver.py        #  83L - LiDAR obstacle stop
│   │   ├── autonomous_driver.py   # 107L - Cone detection + steering
│   │   └── advanced_driver.py     # 154L - PID + centerline
│   │
│   ├── src/                 # ✅ NEW: Modular package structure
│   │   ├── __init__.py
│   │   ├── control/         # Control algorithms
│   │   │   ├── __init__.py
│   │   │   ├── pure_pursuit.py    # get_lookahead_point(), pure_pursuit_steering()
│   │   │   └── speed.py           # estimate_curvature(), calculate_target_speed()
│   │   ├── perception/      # Perception modules
│   │   │   ├── __init__.py
│   │   │   ├── cone_detector.py   # find_cones_filtered(), build_centerline()
│   │   │   ├── slam.py            # SimpleSLAM class
│   │   │   └── cone_classifier.py
│   │   ├── utils/           # Utilities
│   │   │   ├── __init__.py
│   │   │   ├── watchdog.py        # DriveState, StopReason, Watchdog
│   │   │   └── lap_timer.py
│   │   ├── v2x/             # V2X communication
│   │   │   ├── __init__.py
│   │   │   └── rsu.py
│   │   └── drivers/         # Driver implementations
│   │       ├── __init__.py
│   │       ├── competition.py     # Modular CompetitionDriver (uses src.*)
│   │       ├── basic.py
│   │       ├── autonomous.py
│   │       └── advanced.py
│   │
│   ├── config/              # ✅ NEW: Externalized configuration
│   │   └── driver_params.yaml     # All ROS parameters in YAML
│   │
│   ├── launch/              # ✅ NEW: ROS launch files
│   │   └── competition.launch     # Loads params + runs driver
│   │
│   ├── docs/                # ✅ NEW: Architecture documentation
│   │   └── ARCHITECTURE.md
│   │
│   └── tests/
│       └── test_algorithms.py  # 14 unit tests (pytest/unittest)
│
└── submission/              # Mirror of fsds_docker/ for packaging
    └── (same structure)     # Keep synced: rsync -av fsds_docker/ submission/
```

**Total Python LOC:** ~5,100 lines (including modular structure)

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Add new driver | `fsds_docker/src/drivers/` | Extend from existing driver |
| Add control algorithm | `fsds_docker/src/control/` | Pure functions, no ROS deps |
| Add perception module | `fsds_docker/src/perception/` | Cone detection, SLAM |
| Tune driving params | `fsds_docker/config/driver_params.yaml` | YAML config |
| Change Windows IP | `fsds_docker/.env` | `FSDS_HOST_IP=your.ip` |
| Add Python deps | `fsds_docker/Dockerfile` | Line 57-68, `pip3 install` |
| Add ROS packages | `fsds_docker/Dockerfile` | Line 32-44, `apt-get` |
| Run tests | Inside container | `python3 -m pytest tests/ -v` |
| Sync submission | Project root | `rsync -av fsds_docker/ submission/` |

## CODE MAP

### Modular Structure (src/)

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `pure_pursuit_steering` | func | src/control/pure_pursuit.py | Geometric steering calculation |
| `get_lookahead_point` | func | src/control/pure_pursuit.py | Find target point on path |
| `estimate_curvature` | func | src/control/speed.py | Gradient-based curvature |
| `calculate_target_speed` | func | src/control/speed.py | √(a_lat_max / κ) model |
| `apply_rate_limit` | func | src/control/speed.py | Smooth control changes |
| `calculate_throttle` | func | src/control/speed.py | Speed → throttle mapping |
| `DriveState` | enum | src/utils/watchdog.py | TRACKING/DEGRADED/STOPPING |
| `StopReason` | enum | src/utils/watchdog.py | Watchdog stop reasons |
| `Watchdog` | class | src/utils/watchdog.py | Sensor timeout monitoring |
| `find_cones_filtered` | func | src/perception/cone_detector.py | Grid-based O(N) clustering |
| `build_centerline` | func | src/perception/cone_detector.py | Left/right → center path |
| `CompetitionDriver` | class | src/drivers/competition.py | Main driver (modular) |
| `SimpleSLAM` | class | src/perception/slam.py | Occupancy grid + TF |
| `V2X_RSU` | class | src/v2x/rsu.py | Virtual RSU scenarios |

### Legacy Scripts (scripts/ - deprecated backup)

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `CompetitionDriver` | class | scripts/competition_driver.py | Original monolithic driver |
| `SimpleSLAM` | class | scripts/simple_slam.py | Original SLAM |

## KEY PARAMETERS

All parameters externalized to `config/driver_params.yaml`:

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
- **ROS params**: All tunable values in `config/driver_params.yaml`
- **Modular imports**: Use `from src.control.pure_pursuit import ...`
- **PYTHONPATH**: Must include `/root/catkin_ws/src/fsds_scripts`

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
| Direct edit to scripts/ | Use src/ for new development |

## COMMANDS

```bash
# Start environment
./fsds_docker/start.sh

# Enter dev container
docker exec -it fsds_dev bash

# Run modular driver (inside container)
roslaunch fsds_scripts competition.launch

# Or run directly with PYTHONPATH
export PYTHONPATH=/root/catkin_ws/src/fsds_scripts:$PYTHONPATH
python3 /root/catkin_ws/src/fsds_scripts/src/drivers/competition.py

# Run legacy driver (original monolithic)
python3 /root/catkin_ws/src/fsds_scripts/scripts/competition_driver.py

# Run SLAM (separate terminal)
python3 /root/catkin_ws/src/fsds_scripts/src/perception/slam.py

# Run V2X RSU
python3 /root/catkin_ws/src/fsds_scripts/src/v2x/rsu.py

# Run tests
python3 -m pytest tests/ -v

# Load params manually
rosparam load /root/catkin_ws/src/fsds_scripts/config/driver_params.yaml /competition_driver

# View topics
rostopic list
rostopic echo /fsds/testing_only/odom

# Tune params at runtime
rosparam set /competition_driver/max_speed 8.0

# Sync submission directory
rsync -av --exclude='__pycache__' --exclude='.venv' --exclude='.pytest_cache' fsds_docker/ submission/

# Visualize
docker-compose --profile viz up -d rviz

# Stop all
docker-compose down
```

## TESTING

| Test Suite | Count | Coverage |
|------------|-------|----------|
| Pure Pursuit steering | 3 | Geometric calculations (straight, left, right) |
| Curvature/Speed | 3 | Path analysis, target speed |
| SLAM coordinate transform | 2 | Grid conversion |
| Watchdog state machine | 6 | CONES_LOST, V2X stop, perception, E-stop latch |
| **Total** | **14** | Core algorithms |

Run: `python3 -m pytest tests/ -v` (inside container)

## MODULAR ARCHITECTURE

```
┌─────────────────────────────────────────────────────────────┐
│                    src/drivers/competition.py               │
│                     (CompetitionDriver class)                │
├─────────────────────────────────────────────────────────────┤
│  Imports from:                                               │
│  ├── src.control.pure_pursuit                               │
│  ├── src.control.speed                                      │
│  ├── src.perception.cone_detector                           │
│  └── src.utils.watchdog                                     │
└─────────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────┐    ┌─────────────────┐    ┌──────────────┐
│   control/  │    │   perception/   │    │    utils/    │
├─────────────┤    ├─────────────────┤    ├──────────────┤
│pure_pursuit │    │ cone_detector   │    │  watchdog    │
│   speed     │    │     slam        │    │  lap_timer   │
└─────────────┘    └─────────────────┘    └──────────────┘
```

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
- Container scripts auto-mounted from `./scripts/` and `./src/` - edit on host, run in container
- `src/drivers/competition.py` is the modular version of the competition driver
- `scripts/competition_driver.py` is kept as deprecated backup
- `submission/` directory is a duplicate of `fsds_docker/` for packaging - keep synced
- Service startup order: roscore → fsds_bridge → [dev, lap_timer, cone_classifier]
- All drivers follow 20Hz control loop pattern
- Academic docstrings include algorithm explanations for code review scoring
- PYTHONPATH must be set for modular imports to work
