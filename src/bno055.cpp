#include "libbno055-linux/bno055.hpp"

#include <fcntl.h>
#ifdef __linux__
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#endif
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

namespace bno055lib {

namespace {

inline int16_t read16_le(const uint8_t* buf) noexcept {
    return static_cast<int16_t>(buf[0] | (buf[1] << 8));
}

inline bno055lib::Vector3 parseVector3(const uint8_t* buffer, double scale) noexcept {
    return bno055lib::Vector3{read16_le(buffer) * scale, read16_le(buffer + 2) * scale, read16_le(buffer + 4) * scale};
}

// BNO055 Constants
constexpr uint8_t BNO055_ID = 0xA0;

// Register Map
enum Register : uint8_t {
    PAGE_ID = 0x07,
    CHIP_ID = 0x00,
    ACCEL_REV_ID = 0x01,
    MAG_REV_ID = 0x02,
    GYRO_REV_ID = 0x03,
    SW_REV_ID_LSB = 0x04,
    SW_REV_ID_MSB = 0x05,
    BL_REV_ID = 0x06,

    // Data Registers
    ACCEL_DATA_X_LSB = 0x08,
    MAG_DATA_X_LSB = 0x0E,
    GYRO_DATA_X_LSB = 0x14,
    EULER_H_LSB = 0x1A,
    QUATERNION_DATA_W_LSB = 0x20,
    LINEAR_ACCEL_DATA_X_LSB = 0x28,
    GRAVITY_DATA_X_LSB = 0x2E,
    TEMP = 0x34,

    // Status
    CALIB_STAT = 0x35,
    SELFTEST_RESULT = 0x36,
    INTR_STAT = 0x37,
    SYS_CLK_STAT = 0x38,
    SYS_STAT = 0x39,
    SYS_ERR = 0x3A,

    // Configuration
    UNIT_SEL = 0x3B,
    OPR_MODE = 0x3D,
    PWR_MODE = 0x3E,
    SYS_TRIGGER = 0x3F,
    TEMP_SOURCE = 0x40,
    AXIS_MAP_CONFIG = 0x41,
    AXIS_MAP_SIGN = 0x42,

    // Offsets
    ACCEL_OFFSET_X_LSB = 0x55,
    ACCEL_OFFSET_X_MSB = 0x56,
    ACCEL_OFFSET_Y_LSB = 0x57,
    ACCEL_OFFSET_Y_MSB = 0x58,
    ACCEL_OFFSET_Z_LSB = 0x59,
    ACCEL_OFFSET_Z_MSB = 0x5A,

    MAG_OFFSET_X_LSB = 0x5B,
    MAG_OFFSET_X_MSB = 0x5C,
    MAG_OFFSET_Y_LSB = 0x5D,
    MAG_OFFSET_Y_MSB = 0x5E,
    MAG_OFFSET_Z_LSB = 0x5F,
    MAG_OFFSET_Z_MSB = 0x60,

    GYRO_OFFSET_X_LSB = 0x61,
    GYRO_OFFSET_X_MSB = 0x62,
    GYRO_OFFSET_Y_LSB = 0x63,
    GYRO_OFFSET_Y_MSB = 0x64,
    GYRO_OFFSET_Z_LSB = 0x65,
    GYRO_OFFSET_Z_MSB = 0x66,

    ACCEL_RADIUS_LSB = 0x67,
    ACCEL_RADIUS_MSB = 0x68,
    MAG_RADIUS_LSB = 0x69,
    MAG_RADIUS_MSB = 0x6A
};

// Power Modes
constexpr uint8_t POWER_MODE_NORMAL = 0x00;
[[maybe_unused]] constexpr uint8_t POWER_MODE_LOWPOWER = 0x01;
constexpr uint8_t POWER_MODE_SUSPEND = 0x02;

#ifndef I2C_SLAVE
#define I2C_SLAVE 0x0703
#endif

}  // namespace

class BNO055::Impl {
public:
    uint8_t address_;
    std::string i2c_device_;
    int i2c_fd{-1};
    mutable std::mutex mutex_;
    LoggerCallback logger_;

    // Telemetry Diagnostics
    Diagnostics diagnostics_;

    // Configuration Cache (to restore on reconnect)
    OpMode mode_{OpMode::Config};
    AxisMapConfig axis_map_config_{AxisMapConfig::P1};  // Default P1
    AxisMapSign axis_map_sign_{AxisMapSign::P1};        // Default P1
    bool use_xtal_{false};
    uint8_t unit_sel_val_{0x00};  // Default SI units
    bool has_offsets_{false};
    std::array<uint8_t, 22> offsets_data_{0};

    Impl(uint8_t address, std::string_view i2c_device) : address_(address), i2c_device_(std::string(i2c_device)) {}

    ~Impl() { close_i2c(); }

    void log(LogLevel level, std::string_view msg) {
        if (logger_) {
            logger_(level, msg);
        } else {
            std::string label;
            switch (level) {
                case LogLevel::Debug:
                    label = "[DEBUG]";
                    break;
                case LogLevel::Info:
                    label = "[INFO]";
                    break;
                case LogLevel::Warning:
                    label = "[WARNING]";
                    break;
                case LogLevel::Error:
                    label = "[ERROR]";
                    break;
            }
            std::cerr << "[bno055lib::BNO055] " << label << " " << msg << std::endl;
        }
    }

    bool open_i2c() {
        if (i2c_fd >= 0) {
            return true;
        }
#ifdef __linux__
        i2c_fd = open(i2c_device_.c_str(), O_RDWR);
        if (i2c_fd < 0) {
            log(LogLevel::Error, "Failed to open I2C device: " + i2c_device_);
            return false;
        }
        if (ioctl(i2c_fd, I2C_SLAVE, address_) < 0) {
            log(LogLevel::Error, "Failed to set I2C slave address");
            close(i2c_fd);
            i2c_fd = -1;
            return false;
        }
#else
        // Mock fd for non-Linux platforms
        i2c_fd = 999;
        log(LogLevel::Info, "Mocking I2C interface (non-Linux platform detected)");
#endif
        return true;
    }

    void close_i2c() {
        if (i2c_fd >= 0) {
#ifdef __linux__
            close(i2c_fd);
#endif
            i2c_fd = -1;
        }
    }

    bool reconnect() {
        diagnostics_.reconnect_attempts++;
        log(LogLevel::Warning, "Attempting to reconnect I2C and reinitialize BNO055...");
        close_i2c();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!open_i2c()) {
            return false;
        }

        // Wait boot with timeout
        int timeout = 1000;
        uint8_t id = 0;
        bool boot_ok = false;
        while (timeout > 0) {
            if (read8_raw(CHIP_ID, id)) {
                if (id == BNO055_ID) {
                    boot_ok = true;
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            timeout -= 10;
        }
        if (!boot_ok) {
            log(LogLevel::Error, "Reconnection failed: BNO055 not detected or failed to boot");
            return false;
        }

        // Reset
        if (!write8_raw(SYS_TRIGGER, 0x20)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Wait boot after reset with timeout
        timeout = 1000;
        boot_ok = false;
        while (timeout > 0) {
            uint8_t chip_id = 0;
            if (read8_raw(CHIP_ID, chip_id) && chip_id == BNO055_ID) {
                boot_ok = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            timeout -= 10;
        }
        if (!boot_ok) {
            log(LogLevel::Error, "Reconnection failed: BNO055 did not boot after reset");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Reapply configuration
        if (!write8_raw(PWR_MODE, POWER_MODE_NORMAL)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!write8_raw(PAGE_ID, 0)) return false;

        // Restore external crystal
        uint8_t sys_trigger_val = use_xtal_ ? 0x80 : 0x00;
        if (!write8_raw(SYS_TRIGGER, sys_trigger_val)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Restore unit selection
        if (!write8_raw(UNIT_SEL, unit_sel_val_)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Restore Axis Map & Offsets in CONFIGMODE
        if (!write8_raw(OPR_MODE, static_cast<uint8_t>(OpMode::Config))) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));

        if (!write8_raw(AXIS_MAP_CONFIG, static_cast<uint8_t>(axis_map_config_))) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!write8_raw(AXIS_MAP_SIGN, static_cast<uint8_t>(axis_map_sign_))) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        if (has_offsets_) {
            if (!writeLen_raw(ACCEL_OFFSET_X_LSB, offsets_data_.data(), 22)) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Reapply operating mode
        if (!write8_raw(OPR_MODE, static_cast<uint8_t>(mode_))) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        log(LogLevel::Info, "BNO055 reconnected successfully and state restored");
        return true;
    }

    // Low-level raw methods
    bool write8_raw(uint8_t reg, uint8_t value) {
        if (i2c_fd < 0) return false;
#ifdef __linux__
        uint8_t buffer[2] = {reg, value};
        return ::write(i2c_fd, buffer, 2) == 2;
#else
        (void)reg;
        (void)value;
        return true;
#endif
    }

    bool writeLen_raw(uint8_t reg, const uint8_t* buffer, uint8_t len) {
        if (i2c_fd < 0 || len > 31) return false;
#ifdef __linux__
        uint8_t write_buf[32];  // Stack allocation instead of vector
        write_buf[0] = reg;
        std::memcpy(write_buf + 1, buffer, len);
        return ::write(i2c_fd, write_buf, len + 1) == len + 1;
#else
        (void)reg;
        (void)buffer;
        (void)len;
        return true;
#endif
    }

    bool read8_raw(uint8_t reg, uint8_t& value) {
        if (i2c_fd < 0) return false;
#ifdef __linux__
        uint8_t reg_buf[1] = {reg};
        if (::write(i2c_fd, reg_buf, 1) != 1) return false;
        return ::read(i2c_fd, &value, 1) == 1;
#else
        if (reg == CHIP_ID) {
            value = BNO055_ID;
        } else {
            value = 0;
        }
        return true;
#endif
    }

    // Thread-safe methods with automatic reconnect and retries
    bool write8(uint8_t reg, uint8_t value, int retries = 3) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < retries; ++i) {
            if (i2c_fd < 0 && !open_i2c()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
#ifdef __linux__
            uint8_t buffer[2] = {reg, value};
            if (::write(i2c_fd, buffer, 2) == 2) {
                return true;
            }
#else
            (void)reg;
            (void)value;
            return true;
#endif
            diagnostics_.write_failures++;
            log(LogLevel::Warning, "I2C write failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (reconnect()) {
#ifdef __linux__
            uint8_t buffer[2] = {reg, value};
            if (::write(i2c_fd, buffer, 2) == 2) {
                return true;
            }
#else
            (void)reg;
            (void)value;
            return true;
#endif
        }
        diagnostics_.write_failures++;
        log(LogLevel::Error, "I2C write failed permanently");
        return false;
    }

    bool writeLen(uint8_t reg, const uint8_t* buffer, uint8_t len, int retries = 3) {
        if (len > 31) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        uint8_t write_buf[32];  // Stack allocation instead of vector
        write_buf[0] = reg;
        std::memcpy(write_buf + 1, buffer, len);

        for (int i = 0; i < retries; ++i) {
            if (i2c_fd < 0 && !open_i2c()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
#ifdef __linux__
            if (::write(i2c_fd, write_buf, len + 1) == len + 1) {
                return true;
            }
#else
            return true;
#endif
            diagnostics_.write_failures++;
            log(LogLevel::Warning, "I2C writeLen failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (reconnect()) {
#ifdef __linux__
            if (::write(i2c_fd, write_buf, len + 1) == len + 1) {
                return true;
            }
#else
            return true;
#endif
        }
        diagnostics_.write_failures++;
        log(LogLevel::Error, "I2C writeLen failed permanently");
        return false;
    }

    bool read8(uint8_t reg, uint8_t& value, int retries = 3) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < retries; ++i) {
            if (i2c_fd < 0 && !open_i2c()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
#ifdef __linux__
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                if (::read(i2c_fd, &value, 1) == 1) {
                    return true;
                }
            }
#else
            value = 0;
            if (reg == CHIP_ID) value = BNO055_ID;
            return true;
#endif
            diagnostics_.read_failures++;
            log(LogLevel::Warning, "I2C read failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (reconnect()) {
#ifdef __linux__
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                if (::read(i2c_fd, &value, 1) == 1) {
                    return true;
                }
            }
#else
            value = 0;
            if (reg == CHIP_ID) value = BNO055_ID;
            return true;
#endif
        }
        diagnostics_.read_failures++;
        log(LogLevel::Error, "I2C read failed permanently");
        return false;
    }

    bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len, int retries = 3) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < retries; ++i) {
            if (i2c_fd < 0 && !open_i2c()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
#ifdef __linux__
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                if (::read(i2c_fd, buffer, len) == len) {
                    return true;
                }
            }
#else
            std::memset(buffer, 0, len);
            if (reg == QUATERNION_DATA_W_LSB && len >= 8) {
                int16_t w = 16384;
                buffer[0] = w & 0xFF;
                buffer[1] = (w >> 8) & 0xFF;
            }
            return true;
#endif
            diagnostics_.read_failures++;
            log(LogLevel::Warning, "I2C readLen failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (reconnect()) {
#ifdef __linux__
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                if (::read(i2c_fd, buffer, len) == len) {
                    return true;
                }
            }
#else
            std::memset(buffer, 0, len);
            return true;
#endif
        }
        diagnostics_.read_failures++;
        log(LogLevel::Error, "I2C readLen failed permanently");
        return false;
    }
};

BNO055::BNO055(uint8_t i2c_address, std::string_view i2c_device)
    : impl_(std::make_unique<Impl>(i2c_address, i2c_device)) {}

BNO055::~BNO055() = default;

BNO055::BNO055(BNO055&&) noexcept = default;
BNO055& BNO055::operator=(BNO055&&) noexcept = default;

bool BNO055::begin(OpMode mode) {
    if (!impl_->open_i2c()) {
        return false;
    }

    // Detect device with timeout
    int timeout = 1000;
    uint8_t id = 0;
    bool found = false;
    while (timeout > 0) {
        if (impl_->read8(CHIP_ID, id, 1)) {
            if (id == BNO055_ID) {
                found = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        timeout -= 10;
    }

    if (!found) {
        impl_->log(LogLevel::Error, "BNO055 not detected. Found ID: 0x" + std::to_string(id));
        return false;
    }

    // Config Mode
    setMode(OpMode::Config);

    // Reset
    impl_->write8(SYS_TRIGGER, 0x20);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Wait boot after reset with timeout (avoid infinite loop)
    timeout = 1000;
    found = false;
    while (timeout > 0) {
        if (impl_->read8(CHIP_ID, id, 1) && id == BNO055_ID) {
            found = true;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        timeout -= 10;
    }
    if (!found) {
        impl_->log(LogLevel::Error, "BNO055 did not respond after software reset");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Normal Power Mode
    impl_->write8(PWR_MODE, POWER_MODE_NORMAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    impl_->write8(PAGE_ID, 0);

    // Configure UNIT_SEL explicitly to SI Units (0x00 = m/s^2, dps, degrees, Celsius, Windows orientation)
    impl_->unit_sel_val_ = 0x00;
    impl_->write8(UNIT_SEL, impl_->unit_sel_val_);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    impl_->write8(SYS_TRIGGER, 0x0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Set Operating Mode
    setMode(mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    return true;
}

void BNO055::setMode(OpMode mode) {
    impl_->mode_ = mode;
    impl_->write8(OPR_MODE, static_cast<uint8_t>(mode));
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

OpMode BNO055::getMode() {
    uint8_t mode = 0;
    if (!impl_->read8(OPR_MODE, mode)) {
        throw IMUError("Failed to get OPR_MODE");
    }
    return static_cast<OpMode>(mode);
}

void BNO055::setAxisRemap(AxisMapConfig config) {
    impl_->axis_map_config_ = config;
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(AXIS_MAP_CONFIG, static_cast<uint8_t>(config));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void BNO055::setAxisSign(AxisMapSign sign) {
    impl_->axis_map_sign_ = sign;
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(AXIS_MAP_SIGN, static_cast<uint8_t>(sign));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void BNO055::setExtCrystalUse(bool use_xtal) {
    impl_->use_xtal_ = use_xtal;
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(PAGE_ID, 0);
    if (use_xtal) {
        impl_->write8(SYS_TRIGGER, 0x80);
    } else {
        impl_->write8(SYS_TRIGGER, 0x00);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

Vector3 BNO055::getAccelerometer() {
    auto val = getAccelerometerNoexcept();
    if (!val) {
        throw IMUError("Failed to read accelerometer data");
    }
    return *val;
}

std::optional<Vector3> BNO055::getAccelerometerNoexcept() noexcept {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(ACCEL_DATA_X_LSB, buffer, 6)) {
        return std::nullopt;
    }
    // 1 m/s^2 = 100 LSB
    return parseVector3(buffer, 1.0 / 100.0);
}

Vector3 BNO055::getMagnetometer() {
    auto val = getMagnetometerNoexcept();
    if (!val) {
        throw IMUError("Failed to read magnetometer data");
    }
    return *val;
}

std::optional<Vector3> BNO055::getMagnetometerNoexcept() noexcept {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(MAG_DATA_X_LSB, buffer, 6)) {
        return std::nullopt;
    }
    // 1 uT = 16 LSB
    return parseVector3(buffer, 1.0 / 16.0);
}

Vector3 BNO055::getGyroscope() {
    auto val = getGyroscopeNoexcept();
    if (!val) {
        throw IMUError("Failed to read gyroscope data");
    }
    return *val;
}

std::optional<Vector3> BNO055::getGyroscopeNoexcept() noexcept {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(GYRO_DATA_X_LSB, buffer, 6)) {
        return std::nullopt;
    }
    // 1 dps = 16 LSB. Convert to rad/s (dps * M_PI / 180.0)
    constexpr double scale = (1.0 / 16.0) * (M_PI / 180.0);
    return parseVector3(buffer, scale);
}

Vector3 BNO055::getEulerAngles() {
    auto val = getEulerAnglesNoexcept();
    if (!val) {
        throw IMUError("Failed to read euler angles");
    }
    return *val;
}

std::optional<Vector3> BNO055::getEulerAnglesNoexcept() noexcept {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(EULER_H_LSB, buffer, 6)) {
        return std::nullopt;
    }
    // 1 degree = 16 LSB. Convert to rad (deg * M_PI / 180.0)
    constexpr double scale = (1.0 / 16.0) * (M_PI / 180.0);
    // buffer order: h(yaw), r(roll), p(pitch)
    return Vector3{read16_le(buffer + 2) * scale, read16_le(buffer + 4) * scale, read16_le(buffer) * scale};
}

Vector3 BNO055::getLinearAcceleration() {
    auto val = getLinearAccelerationNoexcept();
    if (!val) {
        throw IMUError("Failed to read linear acceleration data");
    }
    return *val;
}

std::optional<Vector3> BNO055::getLinearAccelerationNoexcept() noexcept {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(LINEAR_ACCEL_DATA_X_LSB, buffer, 6)) {
        return std::nullopt;
    }
    // 1 m/s^2 = 100 LSB
    return parseVector3(buffer, 1.0 / 100.0);
}

Vector3 BNO055::getGravity() {
    auto val = getGravityNoexcept();
    if (!val) {
        throw IMUError("Failed to read gravity data");
    }
    return *val;
}

std::optional<Vector3> BNO055::getGravityNoexcept() noexcept {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(GRAVITY_DATA_X_LSB, buffer, 6)) {
        return std::nullopt;
    }
    // 1 m/s^2 = 100 LSB
    return parseVector3(buffer, 1.0 / 100.0);
}

Quaternion BNO055::getQuaternion() {
    auto val = getQuaternionNoexcept();
    if (!val) {
        throw IMUError("Failed to read quaternion data");
    }
    return *val;
}

std::optional<Quaternion> BNO055::getQuaternionNoexcept() noexcept {
    uint8_t buffer[8]{0};
    if (!impl_->readLen(QUATERNION_DATA_W_LSB, buffer, 8)) {
        return std::nullopt;
    }
    // 1 = 16384 LSB (scale factor 2^14)
    constexpr double scale = 1.0 / 16384.0;
    return Quaternion{read16_le(buffer) * scale, read16_le(buffer + 2) * scale, read16_le(buffer + 4) * scale,
                      read16_le(buffer + 6) * scale};
}

int8_t BNO055::getTemperature() {
    auto val = getTemperatureNoexcept();
    if (!val) {
        throw IMUError("Failed to get temperature");
    }
    return *val;
}

std::optional<int8_t> BNO055::getTemperatureNoexcept() noexcept {
    uint8_t val = 0;
    if (!impl_->read8(TEMP, val)) {
        return std::nullopt;
    }
    return static_cast<int8_t>(val);
}

Diagnostics BNO055::getDiagnostics() const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    return impl_->diagnostics_;
}

CalibrationStatus BNO055::getCalibrationStatus() {
    uint8_t stat = 0;
    if (!impl_->read8(CALIB_STAT, stat)) {
        throw IMUError("Failed to get calibration status");
    }
    CalibrationStatus status;
    status.sys = (stat >> 6) & 0x03;
    status.gyro = (stat >> 4) & 0x03;
    status.accel = (stat >> 2) & 0x03;
    status.mag = stat & 0x03;
    return status;
}

bool BNO055::getSensorOffsets(Offsets& offsets) {
    std::array<uint8_t, 22> data;
    if (!getSensorOffsets(data)) return false;

    offsets.accel_offset_x = static_cast<int16_t>(data[0] | (data[1] << 8));
    offsets.accel_offset_y = static_cast<int16_t>(data[2] | (data[3] << 8));
    offsets.accel_offset_z = static_cast<int16_t>(data[4] | (data[5] << 8));

    offsets.mag_offset_x = static_cast<int16_t>(data[6] | (data[7] << 8));
    offsets.mag_offset_y = static_cast<int16_t>(data[8] | (data[9] << 8));
    offsets.mag_offset_z = static_cast<int16_t>(data[10] | (data[11] << 8));

    offsets.gyro_offset_x = static_cast<int16_t>(data[12] | (data[13] << 8));
    offsets.gyro_offset_y = static_cast<int16_t>(data[14] | (data[15] << 8));
    offsets.gyro_offset_z = static_cast<int16_t>(data[16] | (data[17] << 8));

    offsets.accel_radius = static_cast<int16_t>(data[18] | (data[19] << 8));
    offsets.mag_radius = static_cast<int16_t>(data[20] | (data[21] << 8));
    return true;
}

bool BNO055::getSensorOffsets(std::array<uint8_t, 22>& calib_data) {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    bool ok = impl_->readLen(ACCEL_OFFSET_X_LSB, calib_data.data(), 22);
    setMode(prev);
    if (ok) {
        impl_->offsets_data_ = calib_data;
        impl_->has_offsets_ = true;
    }
    return ok;
}

void BNO055::setSensorOffsets(const Offsets& offsets) {
    std::array<uint8_t, 22> data;
    data[0] = offsets.accel_offset_x & 0xFF;
    data[1] = (offsets.accel_offset_x >> 8) & 0xFF;
    data[2] = offsets.accel_offset_y & 0xFF;
    data[3] = (offsets.accel_offset_y >> 8) & 0xFF;
    data[4] = offsets.accel_offset_z & 0xFF;
    data[5] = (offsets.accel_offset_z >> 8) & 0xFF;

    data[6] = offsets.mag_offset_x & 0xFF;
    data[7] = (offsets.mag_offset_x >> 8) & 0xFF;
    data[8] = offsets.mag_offset_y & 0xFF;
    data[9] = (offsets.mag_offset_y >> 8) & 0xFF;
    data[10] = offsets.mag_offset_z & 0xFF;
    data[11] = (offsets.mag_offset_z >> 8) & 0xFF;

    data[12] = offsets.gyro_offset_x & 0xFF;
    data[13] = (offsets.gyro_offset_x >> 8) & 0xFF;
    data[14] = offsets.gyro_offset_y & 0xFF;
    data[15] = (offsets.gyro_offset_y >> 8) & 0xFF;
    data[16] = offsets.gyro_offset_z & 0xFF;
    data[17] = (offsets.gyro_offset_z >> 8) & 0xFF;

    data[18] = offsets.accel_radius & 0xFF;
    data[19] = (offsets.accel_radius >> 8) & 0xFF;
    data[20] = offsets.mag_radius & 0xFF;
    data[21] = (offsets.mag_radius >> 8) & 0xFF;

    setSensorOffsets(data);
}

void BNO055::setSensorOffsets(const std::array<uint8_t, 22>& calib_data) {
    impl_->offsets_data_ = calib_data;
    impl_->has_offsets_ = true;
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));

    impl_->writeLen(ACCEL_OFFSET_X_LSB, calib_data.data(), 22);

    setMode(prev);
}

bool BNO055::saveCalibrationFile(std::string_view filepath) {
    std::array<uint8_t, 22> data;
    if (!getSensorOffsets(data)) {
        impl_->log(LogLevel::Warning, "Failed to retrieve offsets. Sensor might not be configured.");
        return false;
    }

    std::ofstream ofs(std::string(filepath), std::ios::binary);
    if (!ofs) {
        impl_->log(LogLevel::Error, "Failed to open calibration file for writing: " + std::string(filepath));
        return false;
    }

    ofs.write(reinterpret_cast<const char*>(data.data()), 22);
    impl_->log(LogLevel::Info, "Successfully saved calibration to: " + std::string(filepath));
    return true;
}

bool BNO055::loadCalibrationFile(std::string_view filepath) {
    std::ifstream ifs(std::string(filepath), std::ios::binary);
    if (!ifs) {
        impl_->log(LogLevel::Error, "Failed to open calibration file for reading: " + std::string(filepath));
        return false;
    }

    std::array<uint8_t, 22> data;
    ifs.read(reinterpret_cast<char*>(data.data()), 22);
    if (ifs.gcount() != 22) {
        impl_->log(LogLevel::Error, "Invalid calibration file size (expected 22 bytes): " + std::string(filepath));
        return false;
    }

    setSensorOffsets(data);
    impl_->log(LogLevel::Info, "Successfully loaded calibration from: " + std::string(filepath));
    return true;
}

void BNO055::enterSuspendMode() {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(PWR_MODE, POWER_MODE_SUSPEND);
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void BNO055::enterNormalMode() {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(PWR_MODE, POWER_MODE_NORMAL);
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void BNO055::setLogger(LoggerCallback callback) {
    impl_->logger_ = std::move(callback);
}

Vector3 BNO055::getAccelerometerOrDefault() noexcept {
    return getAccelerometerNoexcept().value_or(Vector3{0.0, 0.0, 0.0});
}

Vector3 BNO055::getMagnetometerOrDefault() noexcept {
    return getMagnetometerNoexcept().value_or(Vector3{0.0, 0.0, 0.0});
}

Vector3 BNO055::getGyroscopeOrDefault() noexcept {
    return getGyroscopeNoexcept().value_or(Vector3{0.0, 0.0, 0.0});
}

Vector3 BNO055::getEulerAnglesOrDefault() noexcept {
    return getEulerAnglesNoexcept().value_or(Vector3{0.0, 0.0, 0.0});
}

Vector3 BNO055::getLinearAccelerationOrDefault() noexcept {
    return getLinearAccelerationNoexcept().value_or(Vector3{0.0, 0.0, 0.0});
}

Vector3 BNO055::getGravityOrDefault() noexcept {
    return getGravityNoexcept().value_or(Vector3{0.0, 0.0, 0.0});
}

Quaternion BNO055::getQuaternionOrDefault() noexcept {
    return getQuaternionNoexcept().value_or(Quaternion{1.0, 0.0, 0.0, 0.0});
}

int8_t BNO055::getTemperatureOrDefault() noexcept {
    return getTemperatureNoexcept().value_or(0);
}

Vector3 toEulerDegrees(const Quaternion& q) noexcept {
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double roll_deg = roll * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;
    double yaw_deg = yaw * 180.0 / M_PI;

    if (yaw_deg < 0.0) {
        yaw_deg += 360.0;
    }

    return Vector3{roll_deg, pitch_deg, yaw_deg};
}

}  // namespace bno055lib
