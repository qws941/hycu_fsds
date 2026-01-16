# FSDS AUTONOMOUS DRIVING - PROJECT KNOWLEDGE BASE

**Generated:** 2026-01-16
**Commit:** b1aaa03
**Branch:** master

## OVERVIEW

Docker-based ROS Noetic environment for Formula Student Driverless Simulator (FSDS) autonomous driving competition. Python scripts control vehicle via ROS topics, connecting to Windows-hosted FSDS simulator.

## STRUCTURE

```
fsds_docker/
├── Dockerfile           # ROS Noetic + FSDS build
├── docker-compose.yml   # 5 services: roscore, bridge, dev, rviz, rosbridge
├── .env                 # FSDS_HOST_IP (Windows IP)
├── start.sh             # One-click launcher
├── preflight_check.sh   # Demo rehearsal script
├── dashboard.html       # Web UI dashboard
└── scripts/
    ├── basic_driver.py      # LiDAR obstacle stop
    ├── autonomous_driver.py # Cone detection + steering
    ├── advanced_driver.py   # PID + centerline following
    ├── competition_driver.py # Pure Pursuit + curvature speed (MAIN)
    ├── simple_slam.py       # Occupancy grid SLAM
    ├── v2x_rsu.py           # V2X Virtual RSU (bonus points)
    ├── lap_timer.py         # Lap time measurement
    └── cone_classifier.py   # Cone color classification
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Add new driver | `fsds_docker/scripts/` | Copy `competition_driver.py` as template |
| Change Windows IP | `fsds_docker/.env` | `FSDS_HOST_IP=your.ip` |
| Add Python deps | `fsds_docker/Dockerfile` | Line 52-58, `pip3 install` |
| Add ROS packages | `fsds_docker/Dockerfile` | Line 32-38, `apt-get` |
| Tune driving params | `scripts/*_driver.py` | `__init__` method of driver class |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `BasicDriver` | class | basic_driver.py | Obstacle detection, emergency stop |
| `AutonomousDriver` | class | autonomous_driver.py | Cone grouping, simple steering |
| `AdvancedDriver` | class | advanced_driver.py | PID steering, centerline tracking |
| `CompetitionDriver` | class | competition_driver.py | Pure Pursuit + curvature speed (MAIN) |
| `SimpleSLAM` | class | simple_slam.py | Occupancy grid mapping |
| `DriveState` | enum | competition_driver.py | TRACKING/DEGRADED/STOPPING |
| `find_cones_filtered` | method | competition_driver.py | Z-filter + cluster quality validation |
| `pure_pursuit_steering` | method | competition_driver.py | Geometric path following |
| `estimate_curvature` | method | competition_driver.py | Path curvature calculation |

## KEY PARAMETERS

| Param | Default | File | Effect |
|-------|---------|------|--------|
| `max_throttle` | 0.25 | competition_driver | Max acceleration |
| `min_speed` / `max_speed` | 2.0 / 6.0 m/s | competition_driver | Speed range |
| `max_steering` | 0.4 | competition_driver | Turn sharpness |
| `lookahead_base` | 4.0m | competition_driver | Pure Pursuit lookahead |
| `cones_range_cutoff` | 12.0m | competition_driver | Cone detection range |
| `cone_min_z` / `cone_max_z` | -0.3 / 0.5m | competition_driver | Height filter |

## ROS TOPICS

| Topic | Type | Direction |
|-------|------|-----------|
| `/fsds/control_command` | ControlCommand | Publish (throttle, steering, brake) |
| `/fsds/lidar/Lidar1` | PointCloud2 | Subscribe |
| `/fsds/testing_only/odom` | Odometry | Subscribe |
| `/slam/map` | OccupancyGrid | Publish (SLAM) |
| `/slam/path` | Path | Publish (SLAM) |

## CONVENTIONS

- **No requirements.txt**: Python deps in Dockerfile only
- **No ROS package.xml on host**: Scripts mounted directly, ROS packages built inside container
- **Windows IP required**: Must set `FSDS_HOST_IP` in `.env` before running

## ANTI-PATTERNS

- **Direct pip install in container**: Add to Dockerfile instead
- **Hardcoded IPs**: Use `.env` file
- **Missing rospy.init_node**: Every driver must call in `__init__`

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

# View topics
rostopic list
rostopic echo /fsds/testing_only/odom

# Visualize
docker-compose --profile viz up -d rviz

# Stop all
docker-compose down
```

## NOTES

- Windows FSDS simulator must be running BEFORE `start.sh`
- ROS Bridge connects via port 41451
- Container scripts auto-mounted from `./scripts/` - edit on host, run in container
- `competition_driver.py` is the main submission for the competition
