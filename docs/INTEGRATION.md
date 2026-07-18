# Advanced ROS 2 Integration Guide

For production robotics applications, simply dropping a sensor read into a `rclcpp::TimerBase` is insufficient. Hardware sensors can disconnect, drivers can crash, and I2C buses can lock up.

This guide demonstrates how to integrate `libbno055-linux` using **ROS 2 Lifecycle Nodes (Managed Nodes)** and strict **Quality of Service (QoS)** profiles to build a truly fault-tolerant IMU driver.

---

## 1. The Lifecycle Node (State Machine) Approach

ROS 2 Lifecycle Nodes enforce a strict state machine (`Unconfigured` ➔ `Inactive` ➔ `Active`). Because the BNO055 requires an explicit initialization sequence over I2C, mapping its hardware states directly to the ROS 2 lifecycle states ensures predictable system startup.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <libbno055-linux/bno055.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BNO055DriverNode : public rclcpp_lifecycle::LifecycleNode {
public:
    BNO055DriverNode() : LifecycleNode("bno055_driver") {}

    // ---------------------------------------------------------
    // 1. CONFIGURE: Allocate resources and test I2C connection
    // ---------------------------------------------------------
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Configuring BNO055 Hardware...");
        
        imu_ = std::make_unique<bno055lib::BNO055>(0x28, "/dev/i2c-1");
        
        // Attempt hardware boot into NDOF fusion mode
        if (!imu_->begin(bno055lib::OpMode::NDOF)) {
            RCLCPP_ERROR(get_logger(), "CRITICAL: I2C connection failed. Sensor missing?");
            return CallbackReturn::FAILURE;
        }

        // Load deterministic calibration
        imu_->loadCalibrationFile("/etc/robot_config/bno055_calib.bin");

        // Setup publisher with SENSOR_DATA QoS (Best Effort)
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
        
        return CallbackReturn::SUCCESS;
    }

    // ---------------------------------------------------------
    // 2. ACTIVATE: Start real-time data streaming
    // ---------------------------------------------------------
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        pub_->on_activate();
        
        // Start 100Hz high-priority polling timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&BNO055DriverNode::publish_imu_data, this)
        );
        
        RCLCPP_INFO(get_logger(), "IMU Stream Activated.");
        return CallbackReturn::SUCCESS;
    }

    // ---------------------------------------------------------
    // 3. DEACTIVATE: Pause streaming (hardware remains configured)
    // ---------------------------------------------------------
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        pub_->on_deactivate();
        timer_->cancel();
        RCLCPP_INFO(get_logger(), "IMU Stream Paused.");
        return CallbackReturn::SUCCESS;
    }

    // ---------------------------------------------------------
    // 4. CLEANUP: Fully release I2C file descriptors
    // ---------------------------------------------------------
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        timer_.reset();
        pub_.reset();
        imu_.reset(); // RAII safely closes /dev/i2c-*
        RCLCPP_INFO(get_logger(), "Hardware released.");
        return CallbackReturn::SUCCESS;
    }

private:
    void publish_imu_data() {
        // Strict exception-free path for deterministic timing
        auto quat = imu_->getQuaternionNoexcept();
        auto gyro = imu_->getGyroscopeNoexcept();
        auto accel = imu_->getLinearAccelerationNoexcept();

        if (quat && gyro && accel) {
            auto msg = sensor_msgs::msg::Imu();
            msg.header.stamp = this->now();
            msg.header.frame_id = "imu_link";
            
            msg.orientation.w = quat->w;
            msg.orientation.x = quat->x;
            msg.orientation.y = quat->y;
            msg.orientation.z = quat->z;
            
            msg.angular_velocity.x = gyro->x;
            msg.angular_velocity.y = gyro->y;
            msg.angular_velocity.z = gyro->z;
            
            msg.linear_acceleration.x = accel->x;
            msg.linear_acceleration.y = accel->y;
            msg.linear_acceleration.z = accel->z;
            
            pub_->publish(msg);
        } else {
            // Log telemtry to diagnostics system
            auto diag = imu_->getDiagnostics();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                "I2C Dropout Detected. Reconnect attempts: %d", diag.reconnect_attempts);
        }
    }

    std::unique_ptr<bno055lib::BNO055> imu_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

---

## 2. SensorData QoS (Quality of Service)

Notice the use of `rclcpp::SensorDataQoS()` in the example above. 
For high-frequency sensor streams (like IMUs running at 100Hz), using the default ROS 2 QoS (`RELIABLE`) is an anti-pattern. If the network experiences temporary packet loss (e.g., over Wi-Fi to a ground station), `RELIABLE` will buffer and attempt to retransmit old IMU data, leading to massive latency spikes in your Kalman Filters.

`SensorDataQoS` defaults to `BEST_EFFORT`. It guarantees that your robot's state estimator will only ever process the *newest, most up-to-date* orientation, instantly dropping stale packets.

---

## 3. Registering with rosdep

To make your package easily buildable by others, declare this library as a system dependency in your `package.xml`:
```xml
<depend>libbno055-linux</depend>
```

When users run `rosdep install --from-paths src -y`, `rosdep` will automatically fetch and install this library via `vcpkg`, `apt`, or source, depending on how you distribute your final software stack.

---

## 4. Building and Running the ROS 2 Examples

The library comes with two pre-built ROS 2 node implementations in the `src/ros2/` directory:
- **`bno055_publisher_node`**: A standalone ROS 2 publisher node optimized for zero-copy intra-process communication.
- **`bno055_lifecycle_publisher_node`**: A managed LifecycleNode that supports state transitions and low-power hardware state mapping.

### 4.1. Prerequisites

Ensure you have ROS 2 (e.g., Humble, Jazzy) installed and sourced in your terminal:
```bash
source /opt/ros/humble/setup.bash
```

### 4.2. Building inside a ROS 2 Workspace

Since this repository contains a `package.xml`, it is compatible with `colcon` and can be built directly inside a ROS 2 workspace.

1. Create a workspace directory (if you do not have one):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
2. Clone or symlink this repository into `~/ros2_ws/src/`.
3. Resolve dependencies using `rosdep`:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -y
   ```
4. Build the package:
   ```bash
   colcon build --packages-select libbno055_linux
   ```
5. Source the workspace:
   ```bash
   source install/setup.bash
   ```

### 4.3. Running the Nodes

You can launch and configure the nodes using the provided launch file and parameters YAML file.

#### A. Launching via Launch File (Recommended)
You can launch either of the node types (`standard` or `lifecycle`) and customize their parameters using the ROS 2 Launch system.

*   **Standard Zero-Copy Node (Default)**:
    ```bash
    ros2 launch libbno055_linux bno055_launch.py node_type:=standard
    ```
*   **Lifecycle Node**:
    ```bash
    ros2 launch libbno055_linux bno055_launch.py node_type:=lifecycle
    ```

To customize parameters, copy and edit the installed template `config/bno055_params.yaml`, then load it via launch:
```bash
ros2 launch libbno055_linux bno055_launch.py params_file:=/path/to/your/custom_params.yaml
```

#### B. Direct Command Line (Alternative)
*   **Standard Zero-Copy Node**:
    ```bash
    ros2 run libbno055_linux bno055_publisher_node
    ```
    You can override parameters directly via CLI:
    ```bash
    ros2 run libbno055_linux bno055_publisher_node --ros-args -p device:="/dev/i2c-2" -p address:=40
    ```
*   **Lifecycle Managed Node**:
    ```bash
    # Terminal 1: Run the node
    ros2 run libbno055_linux bno055_lifecycle_publisher_node
    
    # Terminal 2: Trigger state transitions to start publishing
    ros2 lifecycle set /bno055_lifecycle_publisher_node configure
    ros2 lifecycle set /bno055_lifecycle_publisher_node activate
    ros2 lifecycle set /bno055_lifecycle_publisher_node deactivate
    ```

### 4.4. Verification

Verify that the IMU data is streaming correctly:
```bash
ros2 topic echo /imu/data
```

---

## 5. Linux Kernel Tuning & Hardware Optimization Guide

To minimize latency and achieve deterministic execution for state estimation filters (such as EKF), you must eliminate latencies at both the hardware (I2C/UART bus) and OS kernel (scheduler jitter) levels.

### 5.1. Hardware Bus Speed Tuning

#### A. Overclocking the I2C Bus (Fast Mode 400kHz)
By default, the I2C bus on Raspberry Pi and other Single Board Computers (SBCs) is clocked at 100kHz. Under 100kHz, transmitting 18 bytes of burst raw sensor data takes over 2 milliseconds. Overclocking the I2C bus to 400kHz reduces transaction latency to ~450 microseconds.

For Raspberry Pi (Raspbian/Ubuntu), edit `/boot/firmware/config.txt` (or `/boot/config.txt`):
```ini
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000
```
Apply the changes and reboot:
```bash
sudo reboot
```

#### B. UART Driver Latency Optimization (FTDI Low Latency Mode)
If using a USB-to-UART bridge (e.g., FTDI / CP210X) for UART mode, the Linux FTDI driver buffers data for 16 milliseconds by default to optimize USB packets. This is unacceptable for high-frequency EKF loops.

Force the driver to flush immediately by setting the low latency flag on the serial device:
```bash
# Install setserial utility
sudo apt-get install setserial -y

# Configure the port to low latency mode
sudo setserial /dev/ttyUSB0 low_latency
```

---

### 5.2. Kernel & CPU Jitter Tuning

#### A. Real-Time Kernel Patch (PREEMPT_RT)
Standard Linux kernels are non-preemptible, meaning a high-frequency sensor thread can be stalled indefinitely by internal kernel interrupts or background disk writes.
* For absolute determinism, compile or install a kernel patched with the **`PREEMPT_RT`** real-time patch.
* Check if real-time preemption is active:
  ```bash
  uname -a | grep -i PREEMPT
  # Outputs should contain "PREEMPT RT"
  ```

#### B. Isolating CPU Cores for the EKF Node
Prevent the Linux scheduler from moving other background processes onto the CPU core where your high-rate IMU polling node is running.

Edit the boot cmdline (e.g., `/boot/firmware/cmdline.txt` on Raspberry Pi):
```text
# Isolate CPU core 3 from general user-space processes
isolcpus=3
```
After rebooting, launch your ROS 2 node bound specifically to the isolated core:
```bash
taskset -c 3 ros2 run libbno055_linux bno055_publisher_node
```

#### C. Setting Real-Time Thread Priorities (SCHED_FIFO)
To ensure the polling thread within the `libbno055-linux` library preempts other processes, elevate the thread scheduling priority to real-time FIFO.

Give your user account permission to run real-time threads. Edit `/etc/security/limits.conf`:
```text
# Allow robot-user to allocate real-time priority up to 99
robot-user   rtprio   99
robot-user   memlock  unlimited
```

Within your custom C++ execution nodes, prioritize your thread using POSIX thread API:
```cpp
#include <pthread.h>

void elevate_thread_to_rt(std::thread& th) {
    sched_param sch_params;
    sch_params.sched_priority = 80; // High real-time priority
    pthread_setschedparam(th.native_handle(), SCHED_FIFO, &sch_params);
}
```

---

## 6. Hardware Sampling Limits & Aliasing Warnings (EKF Tuning)

When tuning your state estimation filters (such as EKF), setting the correct sensor sampling rates is critical. Setting the polling rate higher than the physical update rate of the sensor creates **data aliasing** and breaks covariance calculations.

### 6.1. Sensor Hardware Update Rate
The Bosch BNO055 has fixed internal hardware sampling frequencies:
* **Gyroscope (Raw)**: **100 Hz**
* **Accelerometer (Raw)**: **100 Hz**
* **Magnetometer (Raw)**: **20 Hz**
* **Fused Quaternion (NDOF)**: **100 Hz**

> [!WARNING]
> **Do not set `publish_rate` or polling frequencies higher than 100.0 Hz.**
> If you poll at 200 Hz, the library will fetch data faster than the sensor registers update. You will read the exact same sensor value twice. In Kalman Filters (EKF), processing duplicate measurements violates the assumption of independent measurement noise, artificially shrinking the covariance and leading to filter divergence or sluggish response.

### 6.2. Communication Bus Constraints
Ensure your physical bus can handle your target frequency without latency accumulation:

* **I2C at 400kHz (Recommended)**: 
  Reading the 18-byte raw data burst takes **~450 microseconds** of physical line time. At 100Hz (10ms period), this uses only **4.5%** of the available bus time, leaving ample headroom for other devices.
* **I2C at 100kHz (Default)**: 
  Reading the 18-byte raw data burst takes **~1.8 milliseconds**. This is fine for 100Hz but eats **18%** of the bus time.
* **UART at 115200 bps**: 
  The raw burst request/response packets take **~2.17 milliseconds** to transmit. Due to POSIX driver write/read overhead, UART mode is physically limited to a maximum rate of **200Hz - 300Hz**. Polling at 100Hz is highly stable.

---

## 7. Integration with `robot_localization` (ROS 2 EKF)

The standard EKF node in the **`robot_localization`** package is commonly used to fuse raw wheel odometry with IMU data. This section provides a production YAML parameter configuration to fuse the raw, unfiltered IMU outputs (`/imu/raw`) generated by `libbno055_linux`.

### 6.1. EKF Configuration (`ekf.yaml`)
Create an `ekf.yaml` config file. The IMU input vector uses the raw data (`imu/raw`), fusing Z-axis angular velocity (Yaw rate) and X-axis linear acceleration:

```yaml
ekf_filter_node:
  ros__parameters:
    use_sim_time: false
    frequency: 30.0
    two_d_mode: true
    publish_tf: true

    map_frame: map              # System global frame
    odom_frame: odom            # Local odometry frame
    base_link_frame: base_link  # Robot base frame
    world_frame: odom

    # Fusing Wheel Odometry
    odom0: /odom
    odom0_config: [true,  true,  false, # X, Y, Z position
                   false, false, true,  # Roll, Pitch, Yaw orientation
                   true,  false, false, # X, Y, Z velocity
                   false, false, true,  # Roll, Pitch, Yaw angular velocity
                   false, false, false] # Accel
    odom0_queue_size: 10
    odom0_differential: false

    # Fusing Raw BNO055 IMU Data (Bypassing internal fusion filter)
    imu0: /imu/raw
    imu0_config: [false, false, false, # Position (IMU doesn't measure position)
                  false, false, false, # Orientation (Fused inside EKF, raw contains no orientation)
                  false, false, false, # Velocities
                  false, false, true,  # Fusing Roll/Pitch/Yaw angular velocity (yaw rate is critical)
                  true,  false, false, # Fusing linear acceleration (X-axis acceleration)
                  false, false, false]
    imu0_queue_size: 20
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true # Let robot_localization remove gravity vector

    # Process and Observation Noise Covariances
    # Make sure you set these values based on your sensor calibration
    process_noise_covariance: [0.05, 0.0,  ... ]
```

### 6.2. ROS 2 Launch File Integration
To run the BNO055 node alongside `robot_localization`, launch them together in Python:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_bno055 = get_package_share_directory('libbno055_linux')
    
    # 1. BNO055 IMU publisher node
    bno055_node = Node(
        package='libbno055_linux',
        executable='bno055_publisher_node',
        name='bno055_node',
        output='screen',
        parameters=[os.path.join(pkg_bno055, 'config', 'bno055_params.yaml')]
    )

    # 2. robot_localization EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_bno055, 'config', 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom/filtered')]
    )

    return LaunchDescription([
        bno055_node,
        ekf_node
    ])
```

---

## 8. Advanced Read Modes & Hardware Wiring

To enable the new advanced features in standard C++ or ROS 2, you must select the appropriate `read_mode` parameter and configure the physical connections.

### 8.1. Comparison of Read Modes

| Read Mode | Data Gathered | I2C Transaction Length | Latency | Thread Model | Best For |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`standard`** | All (Quat, Euler, Accel, Gyro, Mag, Temp, Gravity) | Multiple separate reads | Medium (~3-5ms) | Timer Polling | General ROS 2 navigation, basic visualization, indoor orientation. |
| **`raw_async`** | Raw Burst (Accel, Mag, Gyro) | Single 18-byte sequential read | Low (~450µs) | Background Polling | High-rate state estimation (EKF), custom math models on isolated CPU cores. |
| **`interrupt`** | Raw Burst (Accel, Mag, Gyro) | Single 18-byte sequential read | Extremely Low (~450µs) | GPIO Edge Triggered (IRQ) | Absolute lowest latency and zero polling CPU cycles. Triggers only when sensor has new data. |

---

### 8.2. Hardware Wiring (With INT Pin Support)

For **`interrupt`** mode, you must connect the BNO055's physical **INT (Interrupt)** pin to a GPIO pin on your SBC (e.g., GPIO 24 on Raspberry Pi).

#### I2C Wiring Table (e.g., Raspberry Pi)
| BNO055 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **Vin** | `3.3V` (Pin 1 or 17) | Power supply |
| **GND** | `GND` (Pin 6 or 9) | Ground reference |
| **SDA** | `GPIO 2` (Pin 3) | I2C Data line |
| **SCL** | `GPIO 3` (Pin 5) | I2C Clock line |
| **ADR** | `GND` (or open) | Sets I2C address to `0x28`. Connect to `3.3V` for `0x29`. |
| **INT** | **`GPIO 24` (Pin 18)** | **Hardware Interrupt Line (Required for `interrupt` mode)** |

---

### 8.3. ROS 2 YAML Parameter Configuration
To enable the interrupt mode, add the `read_mode` and `interrupt_gpio_pin` parameters to your `bno055_params.yaml`:

```yaml
bno055_node:
  ros__parameters:
    connection_type: "i2c"
    device: "/dev/i2c-1"
    address: 40                    # 0x28
    
    # Select read mode: "standard", "raw_async", or "interrupt"
    read_mode: "interrupt"
    interrupt_gpio_pin: 24         # GPIO Pin connected to INT
    
    publish_rate: 100.0            # Used for standard/raw_async. Ignored in interrupt mode.
    frame_id: "imu_link"
```

---

## 9. Ultra-Extreme Tuning for Hardware Limits

When pushing state estimation latency to the absolute theoretical limit, you must tune the physical bus transactions to eliminate CPU blocking and transmission overhead.

### 9.1. Enabling I2C DMA (Direct Memory Access)
By default, the Linux kernel uses CPU-driven polling to handle I2C bytes. Reading the 18-byte raw sensor burst occupies the CPU during the transaction, causing minor timing jitter in high-rate control loops.
* **DMA Optimization**: Offloads the I2C transfer to a dedicated DMA channel. The hardware directly transfers the 18 bytes from the I2C controller into RAM, waking the CPU only once the entire burst is complete.
* **Configuration (Raspberry Pi)**:
  Edit `/boot/firmware/config.txt` (or `/boot/config.txt`):
  ```ini
  # Force enable DMA on the Broadcom I2C controller
  dtoverlay=i2c-dma
  ```
  Reboot to apply: `sudo reboot`.

---

### 9.2. Forcing I2C Repeated Start Conditions
A standard I2C register read involves two distinct operations: a Write (sending the register address) and a Read (fetching data), separated by a "Stop" and "Start" bus condition. 
* **The Flaw**: Stopping the bus allows other high-priority multi-master I2C devices to intercept the bus, creating latency spikes.
* **The Solution**: Use a **Repeated Start** condition. This glues the Write and Read transactions together without releasing the bus.
* **Configuration (Ubuntu/Debian Linux)**:
  Configure the kernel module parameter for the design-ware or BCM I2C controller:
  ```bash
  # Enable repeated starts globally in the kernel I2C module
  sudo sh -c 'echo "Y" > /sys/module/i2c_bcm2835/parameters/combined'
  ```
  To make this persistent across boots, create `/etc/modprobe.d/i2c.conf`:
  ```text
  options i2c_bcm2835 combined=Y
  ```

---

### 9.3. UART Overclocking (921,600 bps)
If your system requires UART (serial) rather than I2C, the default `115200 bps` baudrate is a bottleneck, taking **~2.17ms** per 18-byte read.
* **BNO055 Limit**: The BNO055 hardware supports overclocking the UART interface to **921,600 bps** when configured via its physical hardware configuration pins (`PS0` / `PS1`).
* **Latency Reduction**:
  * `115200 bps` ➔ **2.17 ms** transaction latency.
  * `921600 bps` ➔ **0.27 ms** (270 microseconds) transaction latency.
* **Configuration**:
  Modify your application launch parameters or ROS 2 YAML parameter file:
  ```yaml
  bno055_node:
    ros__parameters:
      connection_type: "uart"
      uart_port: "/dev/ttyUSB0"
      uart_baudrate: 921600        # Overclocked speed
  ```
  Ensure you also configure your USB-to-UART bridge (e.g. CP2102N / FT232RL) to support this baudrate.
  *(Note: Apply the `setserial /dev/ttyUSB0 low_latency` optimization described in Section 5.1 alongside this settings.)*

---

### 9.4. Physical Sub-Sensor Overclocking (1kHz Accel / 2kHz Gyro ODR)
When BNO055 runs in standard fusion modes (like `NDOF`), the sensor output rate is limited to 100Hz. However, when using the sensor in **`OpMode::AMG` (Raw Sensor Mode)**, you can unlock and overclock the physical sensing silicon:
* **Automatic Application**: The library automatically configures BNO055 Page 1 registers upon booting into `AMG` mode.
* **Gyroscope Overclocking**: Overrides the default ODR. Configures Gyroscope Output Data Rate (ODR) to **2000 Hz** and sets its internal filter bandwidth to **523 Hz**.
* **Accelerometer Overclocking**: Configures Accelerometer Output Data Rate (ODR) to **1000 Hz** (1kHz).
* **Usage**: Best when combined with high-frequency EKF algorithms running in custom nodes, reducing physical sensor integration delay to less than 1 millisecond.
