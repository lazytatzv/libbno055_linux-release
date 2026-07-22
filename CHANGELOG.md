# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.7.1] - 2026-07-22

### Added & Performance
- **ROS 2 CallbackGroup Isolation**: Fully isolated high-frequency control/sensor callbacks from 1Hz diagnostics and services using dedicated `MutuallyExclusive` CallbackGroups across all driver and controller nodes.
- **MultiThreadedExecutor**: Upgraded all standalone Node `main()` entrypoints to `rclcpp::executors::MultiThreadedExecutor` for non-blocking parallel execution.
- **Slew-Rate Limiter**: Added `max_slew_rate` parameter to `HeadingController` to constrain angular acceleration, protecting motor gears and preventing wheel slip shock.
- **Outlier Validation**: Added `isValidQuat` helper to reject corrupted or NaN/Inf IMU orientation data before PID calculation.
- **Real-time Priority Elevation**: Added `trySetRealtimePriority` helper to gracefully elevate Linux thread scheduling to `SCHED_FIFO` (Priority 80-85).

## [1.7.0] - 2026-07-22

### Added
- **Heading PID Controller (`bno055lib::HeadingController`)**: Production-grade, zero-allocation C++17 heading controller for mobile robot straight-line driving and orientation lock.
- **Theoretical Optimal Control Architecture**: Upgraded with Trapezoidal (Tustin) Rule Integration, 1st-order Low-Pass Filtered Gyro Rate D-term, Kinematic Feedforward (FF) coupling, and Micro-Deadband smoothing.
- **Production ROS 2 Nodes**:
  - `bno055_heading_control_node`: Standard Composable Component node with Zero-Copy transport.
  - `bno055_lifecycle_heading_control_node`: Managed Lifecycle node for Nav2 `lifecycle_manager` integration.
- **Safety Watchdog Timeout**: Automatic Zero Velocity command generation when input velocity (`cmd_vel_in`) is lost to prevent runaway accidents.
- **IMU Fail-Safe Passthrough**: Automatic 100% velocity passthrough when IMU data is offline or disconnected, ensuring robot movement never stops.
- **Direct Quaternion Update**: Overloaded API allowing callers to pass `Quat` directly without manual Euler conversion.
- **Dedicated Parameter YAML**: Added [`config/heading_control_params.yaml`](config/heading_control_params.yaml) and simplified 1-command startup.

## [1.6.2] - 2026-07-22

### Fixed
- **Rust crates.io Publish**: Fixed a dirty git tree issue in GitHub Actions by explicitly updating `Cargo.lock` alongside `Cargo.toml` version bumps.

## [1.6.1] - 2026-07-22

### Fixed
- **ROS 2 Standalone Build**: Fixed a compilation error where standalone executables attempted to include `rclcpp_components` headers without having the dependency declared. The component registration macro is now properly guarded by `#ifdef BNO055_ROS2_BUILDING_COMPONENT`.

## [1.6.0] - 2026-07-22

### Added
- **Hardware Overclocking**: Added automatic 1kHz/2kHz hardware ODR overclocking in `amg` mode (raw mode) by configuring Page 1 sensor registers.
- **UART Low Latency Mode**: Introduced `uart_low_latency` flag (via `ASYNC_LOW_LATENCY` ioctl) to bypass the default 16ms USB packet buffer delay in Linux FTDI drivers.
- **Real-time Scheduling**: Added `thread_priority` parameter to elevate the C++ polling thread to OS-level `SCHED_FIFO` real-time priority.
- **EKF Pro Tuning Profiles**: Completely overhauled `bno055_params.yaml` to include professional EKF tuning guides, matrix layout formatting, and sensible default covariances.
- **Covariance Tuning Guide**: Appended a comprehensive "How to Determine IMU Covariance" guide to `docs/INTEGRATION.md`.

### Fixed
- **ROS 2 REP-145 Compliance**: Fixed `/imu/raw` topic to correctly publish `-1.0` in `orientation_covariance[0]` to indicate the absence of orientation data, preventing EKF divergence.
- **ROS 2 Buildfarm Error**: Corrected `rclcpp_components` dependency declaration in `package.xml` and `CMakeLists.txt` to fix Jenkins CI builds.
- **macOS CI Build**: Wrapped Linux-specific `<linux/serial.h>` imports inside `#ifdef __linux__` to fix macOS compilation on GitHub Actions.
- **README Structure**: Refined `README.md` to be significantly more concise and developer-friendly.

## [1.5.6] - 2026-07-22

### Added
- Setup automated PyPI publishing (sdist and wheels via cibuildwheel) in GitHub Actions.
- Added `MANIFEST.in` to properly include C++ source and header files in the Python source distribution.

## [1.5.4] - 2026-07-21

### Added
- Added Highlights and Feature Comparison Table ("Why libbno055-linux?") to `README.md`.
- Customized `rust/README.md` specifically for crates.io and Rust developers with absolute link targets.

## [1.5.3] - 2026-07-21

### Added
- Added Python, Ubuntu, and Codecov Coverage badges to `README.md`.

## [1.5.2] - 2026-07-21

### Fixed
- Added Section 10 Rust Integration Guide (`use libbno055`) with production 100Hz control loop example to `docs/INTEGRATION.md`.

## [1.5.1] - 2026-07-21

### Fixed
- Updated Rust documentation in `rust/README.md` to reference published `crates.io` dependency (`cargo add libbno055`).

## [1.5.0] - 2026-07-21

### Added
- Added native C API interface (`bno055_c.h` / `src/bno055_c.cpp`) and `c_demo` sample program.
- Added Pybind11 Python bindings (`import libbno055`), `setup.py`, and `pyproject.toml` for `pip install` support.
- Added safe and idiomatic Rust crate bindings (`use libbno055`) in `rust/` with `build.rs` and `demo` example.
- Updated documentation and architecture diagrams for multi-language FFI integration.

## [1.4.3] - 2026-07-18

### Fixed
- Removed unused `use_interrupt` parameter variable in publisher node to clean up compiler warnings.

## [1.4.2] - 2026-07-18

### Fixed
- Resolved unused return value warnings across I2C and file I/O operations.
- Added missing system headers (`<cstdint>`, `<utility>`) for broad compiler support.

## [1.4.1] - 2026-07-17

### Added
- Introduced `benchmark_imu` example utility to measure burst read latency, scheduling jitter, and standard deviation to validate real-time CPU isolation.

### Fixed
- Fixed compilation errors in `BNO055LifecyclePublisherNode` (incorrect logger call in `on_deactivate` and private destructor).
- Explicitly included `lifecycle_msgs` headers (`state.hpp` and `transition.hpp`) to fix compilation on ROS 2 Jazzy.
- Fixed missing standard library headers (`<mutex>`, `<condition_variable>`, `<algorithm>`) in `benchmark_imu` example.
- Fixed a recursive double-acquisition deadlock in `BNO055MockTest.EKFRawBurstReadingAndAsync` unit test.
- Fixed GitHub Actions release asset upload conflicts in release matrix jobs by appending OS suffixes to package names.
- Optimized `clang-tidy` target properties to selectively run checks only on the core library.

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
