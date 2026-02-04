# AUTONOMOUS - MAIN DEVELOPMENT DIRECTORY

**Parent:** See `../../AGENTS.md` for project overview

## OVERVIEW

Primary development directory for FSDS autonomous driving. Docker-based ROS Noetic environment.

## STRUCTURE

```
autonomous/
├── driver/
│   └── competition_driver.py   # Main ROS node (1044L)
├── modules/
│   ├── control/                # Pure Python (no ROS)
│   │   ├── pure_pursuit.py     # Steering algorithm (96L)
│   │   └── speed.py            # Speed control (190L)
│   ├── perception/
│   │   ├── cone_detector.py    # Grid BFS clustering (227L)
│   │   ├── slam.py             # Occupancy grid (262L)
│   │   └── cone_classifier.py  # Color classification (300L)
│   └── utils/
│       ├── watchdog.py         # State machine (294L)
│       └── lap_timer.py        # Timing (207L)
├── config/
│   └── params.yaml             # All tunable parameters (85L)
├── tests/
│   └── test_algorithms.py      # Unit tests (846L, 29 methods)
├── Dockerfile                  # Build (pinned deps)
├── docker-compose.yml          # 3 services: roscore, fsds_bridge, autonomous
├── .env                        # FSDS_HOST_IP (simulator IP)
├── start.sh                    # One-click launcher
└── entrypoint.sh               # Container init
```

## WHERE TO LOOK

| Task | File | Notes |
|------|------|-------|
| Add new control algorithm | `modules/control/` | Keep pure (no ROS) |
| Add LiDAR processing | `modules/perception/` | Follow existing patterns |
| Tune parameters | `config/params.yaml` | Runtime: `rosparam set` |
| Add Python dependency | `Dockerfile` | `pip3 install` section |
| Change simulator IP | `.env` | `FSDS_HOST_IP=x.x.x.x` |

## MODULE RESPONSIBILITIES

| Module | Purpose | ROS Allowed |
|--------|---------|-------------|
| `control/` | Pure math algorithms | NO - numpy only |
| `perception/` | Sensor processing | NO - numpy only |
| `utils/` | State management, timing | NO - pure Python |
| `driver/` | ROS glue, callbacks | YES |

## CONVENTIONS

- **modules/control/**: Pure functions, no ROS imports, easily testable
- **modules/perception/**: Can use numpy, but ROS types in driver only
- **driver/**: ROS glue code, callbacks, main loop
- **Imports**: Absolute from `/root/catkin_ws/src/autonomous`

## ANTI-PATTERNS

| Forbidden | Do Instead |
|-----------|------------|
| ROS imports in `modules/control/` | Pass data as numpy arrays |
| Global state in modules | Return values, pass state explicitly |
| `import sys; sys.path.append()` | Fix PYTHONPATH in entrypoint |
| Blocking calls in callbacks | Use threading.Lock for data handoff |

## COMMANDS

```bash
# Start (run from this directory)
./start.sh

# Test (inside container)
python3 -m pytest tests/ -v

# Manual run
python3 driver/competition_driver.py

# Run with lap limit
python3 driver/competition_driver.py _target_laps:=1

# View params
rosparam list | grep competition

# Override param at runtime
rosparam set /competition_driver/max_speed 8.0
```

## DOCKER SERVICES

| Service | Role | Health |
|---------|------|--------|
| `roscore` | ROS master | `rostopic list` |
| `fsds_bridge` | FSDS↔ROS bridge | Depends on roscore |
| `autonomous` | Driver container | `sleep infinity` (manual mode) |

## TESTING

```python
# Pre-import ROS mocking (test_algorithms.py pattern)
sys.modules['rospy'] = MagicMock()
sys.modules['tf'] = MagicMock()
# ... then import modules
```

- Framework: unittest (pytest discovers)
- 4 test classes, 29 test methods
- No pytest.ini/conftest.py - standalone

## NOTES

- PYTHONPATH must include `/root/catkin_ws/src/autonomous`
- Container auto-mounts this directory to `/root/catkin_ws/src/autonomous`
- Edit on host, run in container
- params.yaml loaded at node startup, runtime override via `rosparam set`
