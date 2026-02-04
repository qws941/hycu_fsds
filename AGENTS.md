# FSDS AUTONOMOUS DRIVING - PROJECT KNOWLEDGE BASE

**Generated:** 2026-02-04
**Commit:** a9b8f67
**Branch:** main

## OVERVIEW

Docker-based ROS Noetic environment for Formula Student Driverless Simulator (FSDS) autonomous driving competition. Python scripts control vehicle via ROS topics, connecting to FSDS simulator.

## STRUCTURE

```
hycu_fsds/
├── src/autonomous/      # MAIN DEVELOPMENT (use this)
│   ├── driver/          # competition_driver.py (1044L)
│   ├── modules/
│   │   ├── control/     # Pure functions (no ROS deps)
│   │   ├── perception/  # LiDAR, SLAM, cone detection
│   │   └── utils/       # Watchdog, lap timer
│   ├── config/params.yaml
│   ├── tests/           # Unit tests (29 test methods)
│   └── Dockerfile       # Container build
│
├── src/simulator/fsds-linux/  # UE4 FSDS binary (164MB)
├── fsds_docker/         # DEPRECATED - legacy structure
├── submission/          # Distribution package (auto-generated)
├── recordings/          # Recording scripts + FSDS Python client
└── scripts/package.sh   # Build distribution
```

**Total Python LOC:** ~18,800 lines (90 files)

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Main driver logic | `src/autonomous/driver/competition_driver.py` | 1044L, main ROS node |
| Control algorithms | `src/autonomous/modules/control/` | Pure functions, no ROS |
| Perception modules | `src/autonomous/modules/perception/` | Cone detection, SLAM |
| Tune driving params | `src/autonomous/config/params.yaml` | YAML config, runtime override |
| Change simulator IP | `src/autonomous/.env` | `FSDS_HOST_IP=your.ip` |
| Add Python deps | `src/autonomous/Dockerfile` | `pip3 install` section (msgpack-rpc first!) |
| Run tests | Inside container | `python3 -m pytest tests/ -v` |
| Start simulator | `src/simulator/fsds-linux/FSDS.sh` | UE4 binary, Vulkan/llvmpipe |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `CompetitionDriver` | class | driver/competition_driver.py | Main ROS node, state machine |
| `pure_pursuit_steering` | func | modules/control/pure_pursuit.py | Geometric steering calc |
| `calculate_target_speed` | func | modules/control/speed.py | v = sqrt(a_lat_max / k) |
| `find_cones_filtered` | func | modules/perception/cone_detector.py | Grid BFS clustering |
| `SimpleSLAM` | class | modules/perception/slam.py | Occupancy grid mapping |
| `Watchdog` | class | modules/utils/watchdog.py | Sensor timeout detection |
| `DriveState` | enum | modules/utils/watchdog.py | TRACKING/DEGRADED/STOPPING |
| `LapTimer` | class | modules/utils/lap_timer.py | Lap counting, timing |

## KEY PARAMETERS (params.yaml)

| Param | Default | Effect |
|-------|---------|--------|
| `max_speed` | 7.0 m/s | Speed limit |
| `lookahead_base` | 3.5m | Pure Pursuit lookahead |
| `control_loop_rate` | 20 Hz | Main loop frequency |
| `lidar_stale_timeout` | 3.0s | Watchdog threshold |
| `cone_grouping_threshold` | 0.2m | Grid cell size |
| `max_lateral_accel` | 6.0 m/s² | Cornering limit |
| `safety_margin` | 0.3m | Track edge buffer |
| `single_side_offset` | 1.0m | One-sided cone offset |

## ROS TOPICS

| Topic | Type | Direction |
|-------|------|-----------|
| `/fsds/control_command` | ControlCommand | Pub |
| `/fsds/lidar/Lidar1` | PointCloud2 | Sub |
| `/fsds/testing_only/odom` | Odometry | Sub |
| `/v2x/speed_limit` | Float32 | Sub |
| `/lap/count` | Int32 | Pub |

## CONVENTIONS

- **No requirements.txt**: Python deps in Dockerfile only
- **Windows/Linux IP required**: Set `FSDS_HOST_IP` in `.env` before running
- **Thread safety**: `threading.Lock` for callback-main loop data
- **Pure control/**: No ROS dependencies in `modules/control/`
- **Type hints**: Required in all new code
- **Pip order**: `msgpack-rpc-python` MUST install before `airsim`

## ANTI-PATTERNS

| Forbidden | Why |
|-----------|-----|
| `pip install` in container | Add to Dockerfile |
| Hardcoded IPs | Use `.env` file |
| Edit `fsds_docker/` | DEPRECATED - use `src/autonomous/` |
| ROS in `modules/control/` | Keep pure for testing |
| Missing `rospy.init_node` | Every driver needs it |
| `queue_size > 1` for control | Real-time needs `queue_size=1` |
| Generic `except Exception` | Catch specific exceptions |
| `print()` for debug | Use `rospy.logdebug()` |

## COMMANDS

```bash
# Start environment (from src/autonomous/)
./start.sh

# Enter dev container
docker exec -it fsds_autonomous bash

# Run driver (inside container)
python3 /root/catkin_ws/src/autonomous/driver/competition_driver.py

# Run driver with lap limit
python3 driver/competition_driver.py _target_laps:=1

# Run tests
python3 -m pytest tests/ -v

# Build distribution package
./scripts/package.sh

# Record screen (1280x800)
DISPLAY=:0 ffmpeg -f x11grab -video_size 1280x800 -framerate 30 -i :0 \
  -c:v libx264 -preset ultrafast -crf 22 output.mp4
```

## THREE CODEBASES

| Directory | Status | Use |
|-----------|--------|-----|
| `src/autonomous/` | **ACTIVE** | All development here |
| `fsds_docker/` | DEPRECATED | Legacy, do not modify |
| `submission/` | AUTO-GENERATED | Built by package.sh |

## STATE MACHINE

```
TRACKING ─(cones lost 3s)─> DEGRADED ─(cones lost 5s)─> STOPPING
    ^                            │                          │
    └──(cones detected)──────────┴───(sensor recovery)──────┘
```

## KNOWN ISSUES

| Issue | Status | Notes |
|-------|--------|-------|
| docker-compose `service_started` | BUG | Should be `service_healthy` for fsds_bridge |
| `cone_min_z: -1.0` | LOOSE | Tighten to `-0.3` for less ground noise |
| `cone_min_points: 1` | LOOSE | Increase to `3` for noise filtering |
| Cone knockdown | EXPECTED | FSDS PhysX physics - cones fall when hit |
| `host.docker.internal` | LINUX FAIL | Use explicit IP on Linux |

## NOTES

- FSDS simulator must be running BEFORE `start.sh`
- ROS Bridge connects via port 41451
- Container scripts auto-mounted - edit on host, run in container
- All drivers follow 20Hz control loop pattern
- Cone detection: Grid BFS clustering with z-filter [-0.5m, 0.5m]
- TrainingMap is the only working map (CompetitionMap1-3 fail to load)
- Collision box: 100cm(W) × 180cm(L) × 50cm(H) - larger than visual model
