# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.4.1] - 2026-07-17

### Added
- Introduced `benchmark_imu` example utility to measure burst read latency, scheduling jitter, and standard deviation to validate real-time CPU isolation.

### Fixed
- Fixed compilation errors in `BNO055LifecyclePublisherNode` (incorrect logger call in `on_deactivate` and private destructor).
- Explicitly included `lifecycle_msgs` headers (`state.hpp` and `transition.hpp`) to fix compilation on ROS 2 Jazzy.

## [1.4.0] - 2026-07-16

### Added
- Implemented ultra-low-latency 18-byte sequential raw burst-read and background polling async API optimized for EKF state estimation.
- Added hardware interrupts (IRQ) support for async reading.
- Upgraded ROS 2 standard and lifecycle nodes to support high-performance raw async and interrupt driver modes.
- Introduced transport abstraction layer and `MockTransport` for hardware-independent unit testing.
- Added comprehensive integration guides (robot_localization EKF integration, Linux kernel tuning, I2C DMA, and UART overclocking) to `docs/INTEGRATION.md`.

### Changed
- Converted `Vector3` and `Quaternion` to float precision for FPU acceleration.
- Reduced thread synchronization lock granularity to optimize library performance.

### Fixed
- Fixed GCC 13 compiler compatibility by including `<atomic>` and using overloaded std math functions.

## [1.3.2] - 2026-07-16

### Added
- Added `~/reset` service to hardware-reset the IMU and auto-recover the state.
- Added `~/status` (`diagnostic_msgs/DiagnosticStatus`) publisher for full compatibility with standard ROS 2 diagnostics systems.
- Added macOS and Windows build support for POSIX-based UART driver mock compilation.

## [1.3.1] - 2026-07-16

### Fixed
- Fixed duplicate variable declaration in publisher node causing compilation failure.
- Fixed clang-format style violations in C++ files.

## [1.3.0] - 2026-07-16

### Added
- Added `imu/raw` (sensor_msgs/Imu) publisher for unfiltered accelerometer and gyroscope data.
- Added `imu/gravity` (geometry_msgs/Vector3) publisher for gravity vector.
- Added `imu/calib_status` (std_msgs/String) publisher to output calibration status as JSON.
- Added `~/calibration_request` (std_srvs/Trigger) service to query calibration status dynamically.
- Implemented native C++ UART communication backend for USB-to-UART bridges.
- Added new ROS parameters for UART: `connection_type`, `uart_port`, `uart_baudrate`, `uart_timeout`.

## [1.2.3] - 2026-07-16

### Fixed
- Fixed missing member variable declarations for `mag_publisher_` and `temp_publisher_` in the lifecycle node.
- Removed stray python scripts that were accidentally committed.

## [1.2.2] - 2026-07-16

### Fixed
- Fixed CMake keyword signature mismatch (`target_link_libraries` vs `ament_target_dependencies`) for ROS 2 nodes.

## [1.2.1] - 2026-07-16

### Fixed
- Fixed ROS 2 CMake target linking by using `ament_target_dependencies` instead of `target_link_libraries` to prevent build failures on the ROS Buildfarm.

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
