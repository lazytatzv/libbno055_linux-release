# libbno055-linux

[![Build & Test](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml/badge.svg)](https://github.com/lazytatzv/libbno055-linux/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-orange.svg)](https://docs.ros.org/)
[![Version](https://img.shields.io/badge/version-1.7.1-green.svg)](CHANGELOG.md)

A C++17 driver for the Bosch BNO055 9-axis IMU on Linux, with first-class ROS 2 integration and multi-language bindings (Python, C, Rust).

---

## Overview

The core of this library is the **BNO055 hardware driver** and its **ROS 2 driver nodes**.

| Component | Description |
| :--- | :--- |
| **BNO055 C++17 Driver** | Low-level I2C/UART driver with auto-recovery, burst reads, async background threads, and calibration management |
| **ROS 2 Driver Nodes** | Composable and Lifecycle publisher nodes for `imu/data`, `imu/mag`, and `/diagnostics` |
| **Multi-language Bindings** | C99 FFI, Python 3 (`pip`), Rust (`crates.io`) |
| **Heading PID Controller** | Optional utility for straight-line correction on differential-drive robots |

---

## Quick Start

### ROS 2 — IMU Driver Node (One Command)

```bash
ros2 launch libbno055_linux bno055_launch.py
```

Published topics:
- `/imu/data` (`sensor_msgs/Imu`) — Quaternion, angular velocity, linear acceleration
- `/imu/mag` (`sensor_msgs/MagneticField`) — Magnetometer
- `/diagnostics` (`diagnostic_msgs/DiagnosticArray`) — Hardware health

For Nav2 Lifecycle Manager:
```bash
ros2 launch libbno055_linux bno055_launch.py node_type:=lifecycle
```

### Standalone C++17

```cpp
#include "libbno055-linux/bno055.hpp"

bno055lib::BNO055 imu(0x28, "/dev/i2c-1");
imu.begin(bno055lib::OpMode::NDOF);

auto quat = imu.getQuaternionNoexcept();
if (quat) {
    // quat->w, quat->x, quat->y, quat->z
}
```

### Python

```bash
pip install libbno055-linux
```

```python
import libbno055
imu = libbno055.BNO055(address=0x28, device="/dev/i2c-1")
imu.begin(libbno055.OpMode.NDOF)
q = imu.get_quaternion()
```

### Rust

```bash
cargo add libbno055
```

---

## Driver Features

### BNO055 C++17 Hardware Driver

- **I2C and UART** transport backends with POSIX drivers
- **Auto-recovery**: Automatic reconnect on I2C bus lockups, clock-stretching glitches, and UART overrun errors
- **18-byte burst read**: Single sequential read for Accel + Mag + Gyro (`~450µs` at 400kHz I2C)
- **Operating modes**: NDOF, IMU, Compass, AMG, and all BNO055 fusion modes
- **Hardware overclocking**: Accel 1kHz ODR / Gyro 2kHz ODR in AMG mode
- **GPIO interrupt (IRQ)**: Sub-millisecond latency via hardware INT pin using `poll()`
- **Async background threads**: Non-blocking continuous data reads at configurable rate
- **Calibration management**: Save/load calibration offsets to/from binary file
- **Axis remapping**: Full axis config and sign remapping support
- **External crystal**: `setExtCrystalUse()` for improved long-term heading accuracy
- **Suspend/Normal power modes**: Low-power sleep and wake control

### ROS 2 Driver Nodes

- **Composable Component** (`rclcpp_components`) — Zero-copy intra-process transport
- **Managed Lifecycle Node** (`rclcpp_lifecycle`) — Full Nav2 Lifecycle Manager integration
- **Isolated CallbackGroups** — High-frequency sensor publishing (100Hz) is isolated from 1Hz diagnostics
- **MultiThreadedExecutor** — Non-blocking parallel execution
- **Linux `SCHED_FIFO`** — Optional real-time thread priority elevation
- **NaN/Inf Outlier Rejection** — Corrupted sensor frames are dropped before publishing
- **Dynamic Parameters** — PID and timeout parameters can be changed at runtime via `ros2 param set`

---

## Installation

### Option A: apt (ROS 2 binary)

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-libbno055-linux
```

> The `apt` binary is updated periodically by ROS Buildfarm. For the latest features (v1.7+), build from source.

### Option B: colcon (ROS 2 source build) — Recommended

```bash
cd ~/ros2_ws/src
git clone https://github.com/lazytatzv/libbno055-linux.git
cd ~/ros2_ws
colcon build --packages-select libbno055_linux
source install/setup.bash
```

### Option C: CMake (standalone C++ library)

```bash
git clone https://github.com/lazytatzv/libbno055-linux.git
cd libbno055-linux
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

---

## ROS 2 Node Topics & Parameters

### Published Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `imu/data` | `sensor_msgs/Imu` | Quaternion, angular velocity (rad/s), linear acceleration (m/s²) |
| `imu/mag` | `sensor_msgs/MagneticField` | Magnetometer (Tesla) |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | Hardware operational status |

### Parameters (`config/bno055_params.yaml`)

| Parameter | Default | Description |
| :--- | :---: | :--- |
| `device` | `/dev/i2c-1` | I2C device path |
| `address` | `0x28` | I2C slave address |
| `publish_rate_hz` | `100` | Sensor publish rate |
| `frame_id` | `imu_link` | ROS TF frame ID |

---

## Optional: Heading PID Controller

A straight-line correction utility for differential-drive robots is included as an optional component.
It subscribes to `imu/data` and `cmd_vel_in`, and publishes a corrected `cmd_vel`.

```bash
ros2 launch libbno055_linux heading_control_launch.py
```

See [docs/HEADING_CONTROL.md](docs/HEADING_CONTROL.md) for full documentation.

---

## Documentation

- [API Reference — C++ / C / Python / Rust](docs/API_REFERENCE.md)
- [ROS 2 Nodes Architecture](src/ros2/README.md)
- [Heading PID Controller Guide](docs/HEADING_CONTROL.md)
- [Hardware Calibration Guide](docs/CALIBRATION.md)
- [Integration Guide (EKF, Nav2, robot_localization)](docs/INTEGRATION.md)
- [Troubleshooting & FAQ](docs/TROUBLESHOOTING.md)
- [Examples](examples/README.md)

---

## License

[MIT License](LICENSE)
