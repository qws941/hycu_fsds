# FSDS MODULAR SOURCE - KNOWLEDGE BASE

**Location:** `fsds_docker/src/`
**Reference:** See root `AGENTS.md` for project-wide architecture.

## OVERVIEW

Modular source packages for the FSDS autonomous driver. This directory contains the core logic separated into functional domains to ensure testability and maintainability.

## STRUCTURE

- **drivers/**: ROS node implementations. Acts as the "glue" between ROS topics and core logic.
- **control/**: Pure Python control algorithms. Mathematical models for steering and speed.
- **perception/**: World modeling. Includes cone detection, clustering, and SLAM.
- **utils/**: Shared utilities. Watchdog state machines, lap timing, and safety monitoring.
- **v2x/**: Vehicle-to-Everything communication logic and virtual RSU scenarios.

## WHERE TO LOOK

| Logic Type | Location | Implementation Detail |
|------------|----------|-----------------------|
| **Control Logic** | `src/control/` | Pure Python, math-heavy, zero ROS dependencies. |
| **ROS Glue** | `src/drivers/` | `rospy` subscribers, publishers, and parameter loading. |
| **Vision/SLAM** | `src/perception/` | LiDAR pointcloud processing and occupancy grids. |
| **Safety/State** | `src/utils/watchdog.py` | State transitions (TRACKING, DEGRADED, STOPPING). |

## CONVENTIONS

- **Pure Functions**: Logic in `src/control/` must be pure functions or classes with no ROS dependencies to allow unit testing on any machine.
- **Type Hints**: Required for all public function signatures and class members.
- **Modular Imports**: Use absolute imports from the package root: `from src.control.speed import calculate_target_speed`.
- **Thread Safety**: Any data shared between ROS callbacks and the main control loop must be protected by a `threading.Lock`.
- **Inheritance**: New drivers should inherit from base classes in `src.drivers` to maintain consistent behavior.

## ANTI-PATTERNS

- **Legacy Imports**: Never import from the deprecated `scripts/` directory.
- **Global State**: Avoid global variables. Use class attributes with proper locking.
- **ROS in Control**: Do not import `rospy` or use ROS message types inside `src/control/`. Use primitive types or NumPy arrays.
- **Direct Logic in Callbacks**: Keep ROS callbacks thin; delegate processing to perception or control modules.
