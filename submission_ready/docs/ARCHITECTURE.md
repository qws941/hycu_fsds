# FSDS Modular Architecture

## Overview

This document describes the modular code architecture for the FSDS autonomous driving project. The codebase has been refactored from a flat structure (`scripts/`) to a hierarchical Python package structure (`src/`).

## Directory Structure

```
fsds_docker/
├── scripts/                    # Original flat structure (deprecated, kept for compatibility)
│   ├── competition_driver.py   # 785 lines - monolithic driver
│   ├── simple_slam.py
│   ├── v2x_rsu.py
│   └── ...
│
├── src/                        # New modular structure
│   ├── __init__.py
│   ├── control/                # Vehicle control algorithms
│   │   ├── __init__.py
│   │   ├── pure_pursuit.py     # Geometric path following
│   │   └── speed.py            # Curvature-based speed control
│   ├── perception/             # Environment perception
│   │   ├── __init__.py
│   │   ├── cone_detector.py    # Cone detection and centerline
│   │   ├── cone_classifier.py  # Color classification
│   │   └── slam.py             # Occupancy grid SLAM
│   ├── utils/                  # Utilities and safety
│   │   ├── __init__.py
│   │   ├── watchdog.py         # State machine and safety
│   │   └── lap_timer.py        # Performance monitoring
│   ├── v2x/                    # Vehicle-to-Everything
│   │   ├── __init__.py
│   │   └── rsu.py              # Road-Side Unit simulation
│   └── drivers/                # Main driver implementations
│       ├── __init__.py
│       ├── competition.py      # Modular competition driver
│       ├── basic.py
│       ├── autonomous.py
│       └── advanced.py
│
├── config/                     # Configuration files
│   └── driver_params.yaml      # ROS parameters (YAML)
│
├── tests/                      # Unit tests
│   └── test_algorithms.py      # 14 tests for core algorithms
│
└── docs/                       # Documentation
    └── ARCHITECTURE.md         # This file
```

## Module Breakdown

### Control Layer (`src/control/`)

| Module | Functions | Purpose |
|--------|-----------|---------|
| `pure_pursuit.py` | `get_lookahead_point()`, `pure_pursuit_steering()` | Geometric path following |
| `speed.py` | `estimate_curvature()`, `calculate_target_speed()`, `apply_rate_limit()`, `calculate_throttle()` | Speed control |

### Perception Layer (`src/perception/`)

| Module | Functions | Purpose |
|--------|-----------|---------|
| `cone_detector.py` | `find_cones_filtered()`, `build_centerline()`, `update_track_width()` | LiDAR cone detection |
| `cone_classifier.py` | `ConeClassifier` class | Color classification |
| `slam.py` | `SimpleSLAM` class | Occupancy grid mapping |

### Utils Layer (`src/utils/`)

| Module | Classes/Functions | Purpose |
|--------|-------------------|---------|
| `watchdog.py` | `DriveState`, `StopReason`, `Watchdog` | Safety state machine |
| `lap_timer.py` | `LapTimer` class | Performance metrics |

### V2X Layer (`src/v2x/`)

| Module | Classes | Purpose |
|--------|---------|---------|
| `rsu.py` | `V2X_RSU` class | Road-Side Unit simulation |

### Drivers Layer (`src/drivers/`)

| Module | Class | Purpose |
|--------|-------|---------|
| `competition.py` | `CompetitionDriver` | Main competition driver (uses modular imports) |
| `basic.py` | `BasicDriver` | Simple obstacle avoidance |
| `autonomous.py` | `AutonomousDriver` | Cone-following |
| `advanced.py` | `AdvancedDriver` | PID control |

## Data Flow

```
LiDAR PointCloud
      │
      ▼
┌─────────────────────────────────────────────────────────┐
│  cone_detector.find_cones_filtered()                    │
│    → Grid-based O(N) clustering                         │
│    → Height filtering (-0.3m to 0.5m)                   │
│    → Left/Right separation (y > 0 = left)               │
└─────────────────────────────────────────────────────────┘
      │
      ▼
┌─────────────────────────────────────────────────────────┐
│  cone_detector.build_centerline()                       │
│    → Average left/right cone positions                  │
│    → Sort by distance from vehicle                      │
└─────────────────────────────────────────────────────────┘
      │
      ▼
┌─────────────────────────────────────────────────────────┐
│  pure_pursuit.get_lookahead_point()                     │
│    → Select target point at lookahead distance          │
│    → Interpolate if needed                              │
└─────────────────────────────────────────────────────────┘
      │
      ▼
┌─────────────────────────────────────────────────────────┐
│  pure_pursuit.pure_pursuit_steering()                   │
│    → δ = atan2(2L·sin(α), ld)                           │
│    → Clamp to max_steering                              │
└─────────────────────────────────────────────────────────┘
      │
      ▼
┌─────────────────────────────────────────────────────────┐
│  speed.estimate_curvature()                             │
│    → Gradient-based curvature estimation                │
│    → Exponential smoothing                              │
└─────────────────────────────────────────────────────────┘
      │
      ▼
┌─────────────────────────────────────────────────────────┐
│  speed.calculate_target_speed()                         │
│    → v = √(a_lat_max / κ)                               │
│    → Clamp to [min_speed, max_speed]                    │
│    → Apply V2X limits                                   │
└─────────────────────────────────────────────────────────┘
      │
      ▼
┌─────────────────────────────────────────────────────────┐
│  speed.apply_rate_limit()                               │
│    → Smooth throttle/steering changes                   │
└─────────────────────────────────────────────────────────┘
      │
      ▼
   ControlCommand (throttle, steering, brake)
```

## State Machine

```
                    ┌──────────────┐
                    │   TRACKING   │
                    │  (Normal)    │
                    └──────┬───────┘
                           │
            ┌──────────────┴──────────────┐
            │  Cones lost > 1s            │
            │  OR sensor stale            │
            ▼                             ▼
    ┌──────────────┐              ┌──────────────┐
    │   DEGRADED   │──────────────│   STOPPING   │
    │ (Fallback)   │  3s timeout  │   (Safe)     │
    └──────────────┘              └──────────────┘
            │                             │
            │  Cones reacquired           │  V2X stop_zone
            │  AND sensors OK             │  OR E-stop latch
            ▼                             │
    ┌──────────────┐                      │
    │   TRACKING   │◄─────────────────────┘
    └──────────────┘   (Manual reset only for E-stop)
```

## Usage

### Running Modular Driver

```bash
docker exec -it fsds_dev bash
export PYTHONPATH=/root/catkin_ws/src/fsds_scripts:$PYTHONPATH
python3 /root/catkin_ws/src/fsds_scripts/src/drivers/competition.py
```

### Running Legacy Driver

```bash
python3 /root/catkin_ws/src/fsds_scripts/scripts/competition_driver.py
```

### Loading Parameters

```bash
rosparam load /root/catkin_ws/src/fsds_scripts/config/driver_params.yaml /competition_driver
```

## Testing

```bash
cd /root/catkin_ws/src/fsds_scripts
python3 -m pytest tests/ -v
```

## References

- Pure Pursuit: Coulter, R.C. (1992). CMU-RI-TR-92-01
- Occupancy Grid: Thrun, S. (2005). Probabilistic Robotics
- V2X Standards: SAE J2735, ETSI TS 103 301
