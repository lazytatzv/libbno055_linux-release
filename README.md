# libbno055-linux

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Compatible-22314E.svg)
![Linux](https://img.shields.io/badge/OS-Linux-FCC624.svg)
![CMake](https://img.shields.io/badge/CMake-3.10%2B-064F8C.svg)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue.svg)](https://lazytatzv.github.io/libbno055-linux/)
![CI](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml/badge.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

**[View the Official Web Documentation (API, Architecture, Integration Guides)](https://lazytatzv.github.io/libbno055-linux/)**

A thread-safe, dependency-free C++17 library for the BNO055 sensor over I2C on Linux. It provides both a standalone library and ROS 2 nodes.

1. **Standalone C++17 Library**: Zero ROS dependencies. Link it natively in embedded Linux applications (Raspberry Pi, Jetson) using standard CMake.
2. **ROS 2 Nodes**: Provides ROS 2 nodes with zero-copy intra-process communication and Lifecycle Node management.

Designed for control systems that require automatic I2C error recovery and deterministic (`noexcept`) execution.

---

## Key Features

*   **Library + ROS 2**: Cleanly separated hardware logic and ROS interfaces. Use it as a standalone C++ library (`-lbno055-linux`) or launch it as a ROS 2 node.
*   **I2C Error Recovery**: BNO055 is known for I2C lockups on Raspberry Pi due to clock stretching. This library catches `EIO` faults, flushes the bus, and recovers sensor state automatically.
*   **Zero-Copy & noexcept APIs**: Zero heap allocations in hot paths. The ROS 2 node uses `std::unique_ptr` publishing and `noexcept` APIs for zero-copy memory transport and deterministic execution.
*   **C++17 & Dependency-Free**: Pure C++17 implementation. No external dependencies or Arduino wrappers.
*   **Cross-Platform Testing**: Built-in I2C Mocking allows compilation and testing natively on macOS/Windows without physical hardware.

---

## Quick Start

### A. Standalone C++ (No-ROS)

1. **Build and Install**:
   ```bash
   sudo apt update && sudo apt install -y build-essential cmake
   git clone https://github.com/lazytatzv/libbno055-linux.git
   cd libbno055-linux && mkdir build && cd build
   cmake .. -DBUILD_TESTING=OFF
   make -j$(nproc) && sudo make install
   ```

2. **Write your code (`main.cpp`)**:
   ```cpp
   #include <libbno055-linux/bno055.hpp>
   #include <iostream>
   #include <thread>

   int main() {
       // 0x28 is default address, /dev/i2c-1 is default port
       bno055lib::BNO055 imu(0x28, "/dev/i2c-1");

       // Initialize in NDOF fusion mode
       if (!imu.begin(bno055lib::OpMode::NDOF)) {
           std::cerr << "Sensor initialization failed!" << std::endl;
           return 1;
       }

       for (int i = 0; i < 10; ++i) {
           auto q = imu.getQuaternionNoexcept();
            if (q) {
               std::cout << "w: " << q->w << " x: " << q->x << " y: " << q->y << " z: " << q->z << "\n";
            }
           std::this_thread::sleep_for(std::chrono::milliseconds(100));
       }
       return 0;
   }
   ```

3. **Compile and Run**:
   ```bash
   g++ -std=c++17 main.cpp -lbno055-linux -o imu_demo
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
   Launch the high-performance zero-copy node (default) or the lifecycle node:
   ```bash
   ros2 launch libbno055_linux bno055_launch.py
   ```

---

## Detailed Project Documentation

For advanced configuration, integration, and detailed specifications, please refer to the dedicated markdown files in the `docs/` directory or view the [Official Web Documentation](https://lazytatzv.github.io/libbno055-linux/).

*   **[API Reference](docs/API_REFERENCE.md)**: Full class, struct, and function reference for `bno055lib::BNO055`.
*   **[Advanced Integration Guide](docs/INTEGRATION.md)**: Detailed instructions for CMake integration (FetchContent, add_subdirectory), custom ROS 2 parameters YAML, launch configurations, and QoS profiles.
*   **[Architecture & Design Decisions](docs/ARCHITECTURE.md)**: Details on PIMPL compilation firewall, thread safety, auto-recovery state machine, and zero-copy intra-process transport.
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
