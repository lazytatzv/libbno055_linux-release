# Troubleshooting & FAQ

## I2C Permission Denied
**Error**: `Failed to open I2C device: /dev/i2c-1 (Permission denied)`
**Cause**: The user running the binary does not have access to the hardware I2C bus.
**Solution**:
Add your user to the `i2c` group:
```bash
sudo usermod -aG i2c $USER
```
Then log out and log back in.

## BNO055 Clock Stretching (I2C Lockups on Raspberry Pi)
**Error**: `[Warning] Temporary communication dropout` appears frequently.
**Cause**: The Broadcom I2C hardware on Raspberry Pi has a known bug handling the BNO055's clock stretching, causing the kernel driver to timeout.
**Solution**:
1. Lower the I2C baudrate. Edit `/boot/firmware/config.txt` (or `/boot/config.txt`) and set:
   ```text
   dtparam=i2c_arm=on,i2c_arm_baudrate=10000
   ```
   (The BNO055 supports up to 400kHz, but 10kHz to 50kHz is significantly more stable on the Raspberry Pi).
2. Reboot the Pi.

## Sensor Calibration is Lost on Reboot
**Error**: The heading (Yaw) drifts immediately after power-on.
**Cause**: The BNO055 does not have internal non-volatile memory for calibration profiles. It loses its calibration every time it loses power.
**Solution**:
Use the provided `calibrate_imu` example to generate a `bno055_calib.bin` file, then load it in your code immediately after `begin()`:
```cpp
imu.loadCalibrationFile("bno055_calib.bin");
```

## Why isn't the Magnetometer working indoors?
**Error**: Calibration status for `Mag` is always 0 or 1, and heading is highly erratic.
**Cause**: Indoor environments with heavy metal structures, motors, and power lines cause severe magnetic distortion.
**Solution**:
Do not use `NDOF` mode indoors. Switch to `IMUPlus` mode:
```cpp
imu.begin(bno055lib::OpMode::IMUPlus);
```
`IMUPlus` mode ignores the magnetometer and provides a relative heading using only the high-precision gyroscope and accelerometer.

---

## ROS 2 Specific Troubleshooting

### ROS 2 Launch fails to find package share directory
**Error**: `package 'libbno055_linux' not found` or similar.
**Cause**: The workspace has not been sourced, or the package has not been built using `colcon`.
**Solution**:
1. Verify that your terminal has sourced the workspace setup script:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
2. Re-build the workspace and ensure `colcon` successfully builds the package:
   ```bash
   colcon build --packages-select libbno055_linux
   ```

### Intra-Process Zero-Copy Communication is not taking effect
**Error**: High CPU/Memory overhead still present; pointers are not being passed directly.
**Cause**: For ROS 2 to optimize message passing via zero-copy, the publishing node and the subscribing node must:
1. Be loaded into the **same component container** (single process).
2. Utilize `std::unique_ptr` and `std::move()` for publishing (which `bno055_publisher_node` does by default).
3. Both have `use_intra_process_comms` option enabled.
**Solution**:
When creating your pipeline, compose the BNO055 component and your subscriber component inside a single container executable (using `rclcpp_components`) and set `use_intra_process_comms` to `True` in the launch parameters.

### Lifecycle Node fails to publish data immediately
**Error**: The lifecycle node starts up, but `ros2 topic echo /imu/data` outputs nothing.
**Cause**: ROS 2 Lifecycle Nodes start in the `Unconfigured` state. They do not start the timer or open hardware connections until transitioned.
**Solution**:
Trigger the state transitions using the ROS 2 lifecycle CLI:
```bash
ros2 lifecycle set /bno055_lifecycle_publisher_node configure
ros2 lifecycle set /bno055_lifecycle_publisher_node activate
```
You can verify the current node state with:
```bash
ros2 lifecycle get /bno055_lifecycle_publisher_node
```
