# AGENTS.md - Legacy Scripts Knowledge Base

## OVERVIEW
Legacy monolithic scripts used during initial development. **DEPRECATED**.
All active development has moved to the modular `fsds_docker/src` package.

## WARNING
**DO NOT MODIFY** these scripts for new features or bug fixes.
Changes here will not reflect in the modular system. Use `fsds_docker/src` instead.

## STRUCTURE
- `competition_driver.py`: Monolithic implementation of the competition driver.
- `simple_slam.py`: Legacy occupancy grid SLAM.
- `v2x_rsu.py`: Virtual RSU logic.
- `cone_classifier.py`: Original cone color detection.
- `autonomous_driver.py`, `advanced_driver.py`: Early driver iterations.

## CONVENTIONS
- **Monolithic design**: Single files containing ROS node logic, algorithms, and utilities.
- **Mixed Logic**: Control, perception, and V2X code are tightly coupled.
- **Legacy Dependencies**: Relies on relative imports and global variables.

## ANTI-PATTERNS
- **New Files**: Never add new scripts to this directory.
- **Cross-Dependencies**: Do not import these scripts from `src/` modules.
- **Production Use**: Do not use these scripts for competition submission unless explicitly required.
