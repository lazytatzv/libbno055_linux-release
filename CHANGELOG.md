# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.2.0] - 2026-07-15

### Added
- Complete Debian packaging support (CPack and `debian/` directory) for standalone PPA release without ROS dependencies.
- Zero-copy intra-process communication enabled by default for ROS 2 nodes.

### Changed
- Refactored core library to enforce deterministic, `noexcept` execution natively (removed `*OrDefault` APIs in favor of `std::optional`-returning `*Noexcept` APIs).
- Unified standard and high-performance ROS 2 nodes into a single, highly optimized node.
- Re-written documentation to focus strictly on factual, technical features rather than buzzwords.


## [1.1.1] - 2026-07-10

### Changed
- Updated maintainer email to GitHub no-reply address in `package.xml`.

## [1.1.0] - 2026-07-10

### Added
- Three ROS 2 publisher nodes (`bno055_publisher_node`, `bno055_perf_publisher_node`, and `bno055_lifecycle_publisher_node`) in `src/ros2/`.
- ROS 2 Python Launch file (`launch/bno055_launch.py`) and parameters configuration file (`config/bno055_params.yaml`) to dynamically configure and launch the nodes.
- Custom parameterization for QoS overrides (`qos_reliability`, `qos_history_depth`), EKF covariances, and startup calibration autoloading (`calibration_file`).
- Managed diagnostic telemetry publishing to `/diagnostics` using `diagnostic_msgs` for real-time monitoring of I2C health and calibration levels.
- Mock-based ROS 2 node integration tests using GoogleTest in `tests/test_ros2_nodes.cpp`.
- Formatter validation (`clang-format`) checks in the GitHub Actions CI pipeline.
- Flexible GTest offline fallback resolution in `CMakeLists.txt` for offline environments (e.g., ROS buildfarms).
- Comprehensive ROS 2 troubleshooting guides in `docs/TROUBLESHOOTING.md`.

### Changed
- Refactored ROS 2 nodes to eliminate redundant logic (DRY principle) by extracting parameter declarations, logging, covariance filling, and diagnostics building into a shared header `src/ros2/bno055_ros2_common.hpp`.

## [1.0.0] - 2026-07-10

### Added
- Robust, thread-safe, and dependency-free BNO055 library for Linux.
- Simplified API for beginners and visual debugging features.
- `vcpkg` and `Conan` integration support.
- Comprehensive Sphinx-based documentation (using Furo theme) including architecture, integration, and troubleshooting guides.
- GitHub Actions CI workflows and contribution guidelines (`CONTRIBUTING.md`).

### Changed
- Renamed the library and CMake target to `libbno055-linux`.

### Optimized
- Eliminated heap allocations in burst write functions to improve sensor write performance.
