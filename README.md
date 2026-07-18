# libbno055-linux

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Compatible-22314E.svg)
![Linux](https://img.shields.io/badge/OS-Linux-FCC624.svg)
![CMake](https://img.shields.io/badge/CMake-3.10%2B-064F8C.svg)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue.svg)](https://lazytatzv.github.io/libbno055-linux/)
![CI](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml/badge.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

**[View the Official Web Documentation (API, Architecture, Integration Guides)](https://lazytatzv.github.io/libbno055-linux/)**

C++17 BNO055 library and ROS 2 nodes for Linux.

## Features

- **Standalone C++17 library**: Link natively via CMake without ROS dependencies.
- **Native I2C & UART Support**: Fully implements the BNO055 binary protocol for both I2C (`/dev/i2c-*`) and USB-to-UART (`/dev/ttyUSB*`) using fast, low-level POSIX APIs.
- **Full Telemetry Parity**: Publishes standard IMU data, raw unfiltered data (`imu/raw`), gravity vectors (`imu/gravity`), and calibration status via JSON (`~/calib_status`) and `diagnostic_msgs::msg::DiagnosticStatus` (`~/status`).
- **Hardware Reset & Calibration Services**: Provides `~/reset` for software-triggered hardware resets and `~/calibration_request` for dynamic calibration state queries.
- **ROS 2 nodes**: Provides standard and lifecycle node interfaces.
- **Automatic Recovery**: Implements automatic recovery for `EIO` faults, clock stretching issues, and UART `BUS_OVER_RUN` errors.
- **No heap allocations**: Avoids dynamic memory allocation in hot sensor readout paths.
- **Zero-copy publishers**: Implements zero-copy memory transport (`std::unique_ptr`) for ROS 2 publishers.
- **Built-in I2C mocking**: Provides built-in I2C mocking for compilation and testing on macOS/Windows.
- **18-Byte Burst Read (New)**: Sequentially reads 18 bytes of raw sensor outputs (Accel, Mag, Gyro) in a single transaction, reducing bus latency by 3x.
- **Linux GPIO Interrupt (IRQ) Driven Mode (New)**: Bypasses polling loops. Detects rising edge events on the physical INT pin using Linux `poll()`, triggering callbacks at sub-millisecond latency.
- **Single-Precision float Optimizations (New)**: Swapped double-precision floats to 32-bit floats across all vectors and quaternion math to unlock hardware FPU speeds on ARM processors (e.g., Raspberry Pi).

---

## Burst Read & State Estimation (EKF) Features

If you are developing a custom state estimator (Extended Kalman Filter / Complementary Filter) or using **`robot_localization`**, raw sensor throughput and latency determinism are critical.

`libbno055-linux` provides:
1. **18-Byte Burst Read**: Reads Accelerometer, Magnetometer, and Gyroscope raw variables in a single sequential bus transaction (~450µs at 400kHz I2C).
2. **High-Frequency Polling**: Low-jitter background polling threads up to 200Hz.
3. **GPIO Interrupt (IRQ) Driven Mode**: POSIX `poll()` edge event detection on the physical INT pin for sub-millisecond response.

> [!TIP]
> Complete C++ code examples for these APIs, EKF configuration files (`ekf.yaml`) for ROS 2 `robot_localization`, and kernel real-time scheduling setup are located in the **[Advanced Integration & Kernel Tuning Guide](docs/INTEGRATION.md)**.
> For function signatures and types, see the **[API Reference](docs/API_REFERENCE.md)**.

---

## Quick Start

### A. Standalone C++ (No-ROS)

1. **Build and Install**:
   ```bash
   sudo apt update && sudo apt install -y build-essential cmake
   git clone https://github.com/lazytatzv/libbno055-linux.git
   cd libbno055-linux && mkdir build && cd build
   cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=ON
   make -j$(nproc) && sudo make install
   ```

2. **Write your code (`main.cpp`)**:
   ```cpp
   #include <libbno055-linux/bno055.hpp>
   #include <iostream>
   #include <thread>

   int main() {
       // Initialize via I2C (Default)
       // bno055lib::BNO055 imu(0x28, "/dev/i2c-1");

       // OR Initialize via UART
       bno055lib::BNO055::UARTConfig uart_cfg;
       uart_cfg.port = "/dev/ttyUSB0";
       uart_cfg.baudrate = 115200;
       bno055lib::BNO055 imu(uart_cfg);

       // Initialize in NDOF fusion mode
       if (!imu.begin(bno055lib::OpMode::NDOF)) {
           std::cerr << "Sensor initialization failed!\n";
           return 1;
       }

       // Configure automatic calibration loading & saving
       imu.enableAutoCalibration("/etc/robot_config/bno055_offsets.bin");

       for (int i = 0; i < 10; ++i) {
           // Orientation (Euler Angles converted from Quaternion)
           if (auto q = imu.getQuaternionNoexcept()) {
               auto euler = bno055lib::toEulerDegrees(*q);
               std::cout << "Euler (deg): Roll=" << euler.x << " Pitch=" << euler.y << " Yaw=" << euler.z << "\n";
           }

           // Acceleration
           if (auto acc = imu.getAccelerometerNoexcept()) {
               std::cout << "Accel (m/s^2): X=" << acc->x << " Y=" << acc->y << " Z=" << acc->z << "\n";
           }

           // Calibration Status
           auto calib = imu.getCalibrationStatus();
           std::cout << "Calibration: SYS=" << static_cast<int>(calib.sys) 
                     << " GYRO=" << static_cast<int>(calib.gyro) << "\n";

           // Temperature
           std::cout << "Temperature: " << static_cast<int>(imu.getTemperature()) << " C\n\n";

           std::this_thread::sleep_for(std::chrono::milliseconds(100));
       }
       return 0;
   }
   ```

3. **Compile and Run**:
   ```bash
   g++ -std=c++17 main.cpp -lbno055-linux -lpthread -o imu_demo
   ./imu_demo
   ```

---

### B. ROS 2 (colcon workspace)

1. **Clone and Build**:
   ```bash
   # Clone inside your ROS 2 workspace src directory
   cd ~/ros2_ws/src
   git clone https://github.com/lazytatzv/libbno055-linux.git

   # Resolve dependencies and build
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -y
   colcon build --packages-select libbno055_linux
   source install/setup.bash
   ```

2. **Launch with Parameters**:
   Launch the zero-copy node (default) or the lifecycle node:
   ```bash
   ros2 launch libbno055_linux bno055_launch.py
   ```

### C. ROS 2 API Reference

#### Published Topics
| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `~/imu/data` | `sensor_msgs/msg/Imu` | Fused IMU data (Orientation, Angular Velocity, Linear Acceleration). |
| `~/imu/raw` | `sensor_msgs/msg/Imu` | Raw, unfiltered Accelerometer and Gyroscope data. |
| `~/imu/mag` | `sensor_msgs/msg/MagneticField` | Raw Magnetic Field data. |
| `~/imu/temp` | `sensor_msgs/msg/Temperature` | Ambient Temperature data. |
| `~/imu/gravity` | `geometry_msgs/msg/Vector3` | Gravity vector (available in fusion modes). |
| `~/status` | `diagnostic_msgs/msg/DiagnosticStatus` | Hardware and calibration status compatibility topic. |
| `~/calib_status` | `std_msgs/msg/String` | JSON formatted real-time calibration levels (Sys, Gyro, Accel, Mag). |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Standard ROS 2 diagnostics stream (heartbeat, error rates, dropped reads). |

#### Services
| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `~/save_calibration` | `std_srvs/srv/Trigger` | Saves the current calibration offsets to the file specified in parameters. |
| `~/calibration_request` | `std_srvs/srv/Trigger` | Immediately responds with a JSON string of the current calibration status. |
| `~/reset` | `std_srvs/srv/Trigger` | Triggers a software hardware-reset of the BNO055 and reinitializes it automatically. |

#### Core Parameters
| Parameter | Default | Description |
| :--- | :--- | :--- |
| `connection_type` | `"i2c"` | `"i2c"` or `"uart"`. Determines the hardware communication interface. |
| `device` | `"/dev/i2c-1"` | Path to the I2C device node (used if `connection_type` is `"i2c"`). |
| `address` | `40` | I2C address of the sensor (`40` for 0x28, `41` for 0x29). |
| `uart_port` | `"/dev/ttyUSB0"` | Path to the UART device node (used if `connection_type` is `"uart"`). |
| `uart_baudrate`| `115200` | Baudrate for the UART connection. |
| `operation_mode` | `"ndof"` | Sensor fusion mode (`"ndof"`, `"imu_plus"`, `"compass"`, etc.). |
| `publish_rate` | `100.0` | Publishing frequency in Hz. |
| `frame_id` | `"imu_link"` | TF frame ID attached to message headers. |
| `enable_auto_calibration` | `false` | Enable automatic loading and saving of the calibration binary profile. |

---

## Detailed Project Documentation

For advanced configuration, integration, and detailed specifications, please refer to the dedicated markdown files in the `docs/` directory or view the [Official Web Documentation](https://lazytatzv.github.io/libbno055-linux/).

*   **[API Reference](docs/API_REFERENCE.md)**: Full class, struct, and function reference for `bno055lib::BNO055`.
*   **[Advanced Integration & Kernel Tuning Guide](docs/INTEGRATION.md)**: Details on CMake integration, ROS 2 configuration parameters, I2C speed configurations (400kHz), UART driver optimizations, `PREEMPT_RT` scheduler priorities, and isolated CPU cores.
*   **[Architecture & Design Decisions](docs/ARCHITECTURE.md)**: Details on PIMPL compilation firewall, thread safety, auto-recovery state machine, `Transport` DI abstraction, float math FPU optimizations, and lock granularity scopes.
*   **[Sensor Overview & Calibration](docs/SENSOR_OVERVIEW.md)**: Detailed specs on BNO055 fusion modes, sensor coordinate systems, and saving/restoring calibration offsets.
*   **[Troubleshooting & FAQ](docs/TROUBLESHOOTING.md)**: Solutions for I2C permission denied, Raspberry Pi clock stretching lockups, and indoor magnetic interference.

---

## Hardware Configuration (Prerequisites)

Ensure the physical sensor is wired correctly and I2C permissions are set up on your Linux machine.

### 1. Wiring (Raspberry Pi Example)

| BNO055 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **Vin** | `3.3V` (Pin 1 or 17) | Power supply |
| **GND** | `GND` (Pin 6 or 9) | Ground |
| **SDA** | `GPIO 2` (Pin 3) | I2C Data |
| **SCL** | `GPIO 3` (Pin 5) | I2C Clock |
| **ADR** | `GND` (or open) | Sets address to `0x28`. Connect to `3.3V` for `0x29`. |

### 2. Permissions & I2C Enable
1. Enable the I2C interface via `sudo raspi-config` (Interface Options -> I2C).
2. Add your user to the `i2c` group to run programs without `sudo`:
   ```bash
   sudo usermod -aG i2c $USER
   ```
   *Note: Log out and log back in for changes to take effect.*

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
