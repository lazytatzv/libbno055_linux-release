# libbno055-linux API Reference

<details>
<summary><strong>Table of Contents</strong></summary>

- [Namespaces & Types](#namespaces--types)
- [Class BNO055](#class-bno055)
  - [Lifecycle](#lifecycle)
  - [Configuration](#configuration)
  - [Sensor Data (Throwing APIs)](#sensor-data-throwing-apis)
  - [Sensor Data (Exception-free / noexcept APIs)](#sensor-data-exception-free--noexcept-apis)
  - [Diagnostics & Calibration](#diagnostics--calibration)
  - [Power Management](#power-management)
  - [Logging](#logging)
- [Utilities (Class-External)](#utilities-class-external)

</details>

## Namespaces & Types

```cpp
namespace bno055lib {
    // 3D coordinate vector (used for accelerometer, gyroscope, magnetometer, euler, gravity, linear accel)
    struct Vector3 {
        double x;
        double y;
        double z;
    };

    // 3D orientation quaternion representation
    struct Quaternion {
        double w; // Real part
        double x; // Imaginary X
        double y; // Imaginary Y
        double z; // Imaginary Z
    };
    
    // Binary calibration offsets (22 bytes total) for saving/restoring sensor profile
    struct Offsets {
        int16_t accel_offset_x, accel_offset_y, accel_offset_z;
        int16_t mag_offset_x, mag_offset_y, mag_offset_z;
        int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
        int16_t accel_radius, mag_radius;
    };

    // Dynamic calibration status of the sensor (0 = uncalibrated, 3 = fully calibrated)
    struct CalibrationStatus {
        uint8_t sys;   // Overall system calibration status [0-3]
        uint8_t gyro;  // Gyroscope calibration status [0-3]
        uint8_t accel; // Accelerometer calibration status [0-3]
        uint8_t mag;   // Magnetometer calibration status [0-3]
        
        // Returns true if gyro, accel, and mag are fully calibrated (status == 3)
        bool isFullyCalibrated() const;
    };

    // Cumulative telemetry diagnostics tracking error rates for health monitoring
    struct Diagnostics {
        uint32_t write_failures;      // Number of failed register write transactions
        uint32_t read_failures;       // Number of failed register read transactions
        uint32_t reconnect_attempts;  // Number of I2C bus auto-reconnect triggers
    };

    // Operating mode configuration
    enum class OpMode : uint8_t {
        Config = 0X00,          // Configuration mode (required to write map/sign/crystal settings)
        AccOnly = 0X01,         // Non-fusion Accelerometer only
        MagOnly = 0X02,         // Non-fusion Magnetometer only
        GyroOnly = 0X03,        // Non-fusion Gyroscope only
        AccMag = 0X04,          // Non-fusion Accelerometer + Magnetometer
        AccGyro = 0X05,         // Non-fusion Accelerometer + Gyroscope
        MagGyro = 0X06,         // Non-fusion Magnetometer + Gyroscope
        AMG = 0X07,             // Non-fusion Accelerometer + Magnetometer + Gyroscope (raw outputs)
        IMUPlus = 0X08,         // 6-axis Fusion (Acc + Gyro). Yaw relative to boot position. Recommended indoors.
        Compass = 0X09,         // 6-axis Fusion (Acc + Mag). Absolute Yaw.
        M4G = 0X0A,             // 6-axis Fusion (Mag + Gyro).
        NDOF_FMC_Off = 0X0B,    // 9-axis Fusion (Acc + Mag + Gyro) with Fast Magnetometer Calibration disabled
        NDOF = 0X0C             // 9-axis Fusion (Acc + Mag + Gyro) with FMC enabled. Absolute Yaw (North-referenced).
    };

    enum class LogLevel { Debug, Info, Warning, Error };
    using LoggerCallback = std::function<void(LogLevel level, std::string_view message)>;
}
```
### Class BNO055

All functions in the `BNO055` class are thread-safe and protect access to the underlying I2C bus using internal mutexes.

#### Lifecycle

*   **explicit BNO055(uint8_t i2c_address = 0x28, std::string_view i2c_device = "/dev/i2c-1")**
    *   *Parameters*:
        *   `i2c_address`: `uint8_t`. The I2C slave address of the BNO055 (typically `0x28` or `0x29`).
        *   `i2c_device`: `std::string_view`. The Linux file path to the I2C adapter (e.g., `/dev/i2c-1`).
    *   *Returns*: None (Constructor).
    *   *Description*: Creates a BNO055 handler. If compiled on macOS/Windows, it falls back to the mock mode and accepts any mock device name.
*   **bool begin(OpMode mode = OpMode::NDOF)**
    *   *Parameters*:
        *   `mode`: `OpMode`. The target operation mode to start in.
    *   *Returns*: `bool`. `true` if initialization, self-test verification, and mode transition succeeded; `false` on boot timeouts or I2C failures.
    *   *Description*: Power-cycles the sensor, verifies the Chip ID, configures default Axis Maps and Unit settings, and spawns the background auto-recovery thread.

#### Configuration

*   **void setMode(OpMode mode)**
    *   *Parameters*:
        *   `mode`: `OpMode`. The target operation mode to switch into.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on permanent I2C communication failures.
    *   *Description*: Switches the operating mode of the sensor. Handles transition delay requirements automatically.
*   **OpMode getMode()**
    *   *Parameters*: None.
    *   *Returns*: `OpMode`. The active operation mode currently registered on the sensor.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **void setAxisRemap(AxisMapConfig config)**
    *   *Parameters*:
        *   `config`: `AxisMapConfig`. The remap configuration matching the sensor's mounting orientation.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **void setAxisSign(AxisMapSign sign)**
    *   *Parameters*:
        *   `sign`: `AxisMapSign`. The axis sign configuration to adjust rotation/acceleration directions.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **void setExtCrystalUse(bool use_xtal)**
    *   *Parameters*:
        *   `use_xtal`: `bool`. Set to `true` to use the external 32.768kHz crystal oscillator for enhanced fusion stability.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.

#### Sensor Data (Throwing APIs)

These functions query raw data registers and convert them to SI units. They throw `bno055lib::IMUError` on permanent I2C communication loss.

*   **Vector3 getAccelerometer()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis acceleration in meters per second squared (m/s^2).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getMagnetometer()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis magnetic field strength in microteslas (uT).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getGyroscope()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis angular velocity in radians per second (rad/s).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getEulerAngles()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. Roll, Pitch, Yaw in radians (rad). Mapping: `x` = Roll, `y` = Pitch, `z` = Yaw.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getLinearAcceleration()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis linear acceleration (accelerometer excluding gravity) in m/s^2.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getGravity()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis gravity vector in m/s^2.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Quaternion getQuaternion()**
    *   *Parameters*: None.
    *   *Returns*: `Quaternion`. Normalized 3D orientation unit quaternion (w, x, y, z).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **int8_t getTemperature()**
    *   *Parameters*: None.
    *   *Returns*: `int8_t`. Chip temperature in degrees Celsius (C).
    *   *Exceptions*: Throws `bno055lib::IMUError`.

#### Sensor Data (Exception-free / noexcept APIs)

These companion APIs perform the exact same register queries and conversions but never throw exceptions.

*   **std::optional\<Vector3\> getAccelerometerNoexcept() noexcept**
*   **std::optional\<Vector3\> getMagnetometerNoexcept() noexcept**
*   **std::optional\<Vector3\> getGyroscopeNoexcept() noexcept**
*   **std::optional\<Vector3\> getEulerAnglesNoexcept() noexcept**
*   **std::optional\<Vector3\> getLinearAccelerationNoexcept() noexcept**
*   **std::optional\<Vector3\> getGravityNoexcept() noexcept**
*   **std::optional\<Quaternion\> getQuaternionNoexcept() noexcept**
*   **std::optional\<int8_t\> getTemperatureNoexcept() noexcept**
    *   *Parameters*: None.
    *   *Returns*: `std::optional<T>`. Contains the requested struct on success; `std::nullopt` on communication failure.
    *   *Description*: Safety-hardened read path that increments I2C diagnostic counters upon failure without generating CPU exceptions.

    *   *Parameters*: None.
    *   *Returns*: The requested struct directly (`Vector3`, `Quaternion`, or `int8_t`). On temporary bus drops, returns the last cached valid frame (or zero/identity).

#### Diagnostics & Calibration

*   **Diagnostics getDiagnostics() const noexcept**
    *   *Parameters*: None.
    *   *Returns*: `Diagnostics`. The telemetry diagnostic struct containing I2C read/write error counts and reconnection attempts.
*   **CalibrationStatus getCalibrationStatus()**
    *   *Parameters*: None.
    *   *Returns*: `CalibrationStatus`. Current calibration state values (0 to 3) for the system, gyro, accelerometer, and magnetometer.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **bool getSensorOffsets(Offsets& offsets)**
    *   *Parameters*:
        *   `offsets`: `Offsets&` (output). Structure to store the retrieved calibration offsets.
    *   *Returns*: `bool`. `true` if offsets were read and stored successfully; `false` on I2C failure.
*   **bool getSensorOffsets(std::array<uint8_t, 22>& calib_data)**
    *   *Parameters*:
        *   `calib_data`: `std::array<uint8_t, 22>&` (output). Raw byte array to store the 22 bytes of offsets.
    *   *Returns*: `bool`. `true` if read succeeded; `false` on failure.
*   **void setSensorOffsets(const Offsets& offsets)**
    *   *Parameters*:
        *   `offsets`: `const Offsets&`. Calibration offset parameters to load into the sensor registers.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
    *   *Description*: Writes offsets using a single-batch sequential I2C write transaction to minimize bus occupation.
*   **void setSensorOffsets(const std::array<uint8_t, 22>& calib_data)**
    *   *Parameters*:
        *   `calib_data`: `const std::array<uint8_t, 22>&`. Raw 22-byte array containing calibration offsets.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
*   **bool saveCalibrationFile(std::string_view filepath)**
    *   *Parameters*:
        *   `filepath`: `std::string_view`. Destination file path to save the 22-byte profile (e.g., `calib.bin`).
    *   *Returns*: `bool`. `true` if offsets were read and successfully written to disk; `false` on I2C or file I/O errors.
*   **bool loadCalibrationFile(std::string_view filepath)**
    *   *Parameters*:
        *   `filepath`: `std::string_view`. Source path of the 22-byte binary profile.
    *   *Returns*: `bool`. `true` if file read, register upload, and cache storage succeeded; `false` on file I/O or I2C errors.
    *   *Description*: Uploads calibration to the sensor and caches it locally so it can be automatically restored during an I2C hot-reconnect recovery.

#### Power Management

*   **void enterSuspendMode()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
    *   *Description*: Places the sensor into suspend mode to reduce power consumption. Suspends accelerometer, gyroscope, and magnetometer blocks.
*   **void enterNormalMode()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
    *   *Description*: Awakes the sensor from suspend mode back to active normal operation.

#### Logging

*   **void setLogger(LoggerCallback callback)**
    *   *Parameters*:
        *   `callback`: `LoggerCallback`. Callback function of signature `void(LogLevel level, std::string_view message)`.
    *   *Returns*: `void`.
    *   *Description*: Hooks a custom logging function (such as `std::cout` or ROS 2 logging macros) to direct library diagnostics, warnings, and reconnect traces.

### Utilities (Class-External)

*   **Vector3 toEulerDegrees(const Quaternion& q) noexcept**
    *   *Parameters*:
        *   `q`: `const Quaternion&`. The normalized unit quaternion representation of orientation.
    *   *Returns*: `Vector3`. Roll, Pitch, and Yaw in degrees (Mapping: `x` = Roll `[-180, 180]`, `y` = Pitch `[-90, 90]`, `z` = Yaw `[0, 360)`).
    *   *Description*: Utility function to convert Quaternion orientation into human-readable Euler angles in degrees.
