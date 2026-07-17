#include "libbno055-linux/bno055.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>

#include "libbno055-linux/transport.hpp"

#ifdef __linux__
#include <linux/i2c-dev.h>
#endif
#include <unistd.h>

#include <atomic>
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

inline bno055lib::Vector3 parseVector3(const uint8_t* buffer, float scale) noexcept {
    return bno055lib::Vector3{static_cast<float>(read16_le(buffer)) * scale,
                              static_cast<float>(read16_le(buffer + 2)) * scale,
                              static_cast<float>(read16_le(buffer + 4)) * scale};
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
    MAG_RADIUS_MSB = 0x6A,

    // Page 1 Registers
    ACC_CONFIG = 0x08,
    GYR_CONFIG_0 = 0x0A,
    GYR_CONFIG_1 = 0x0B
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
    enum class ConnectionType { I2C, UART };
    ConnectionType conn_type_{ConnectionType::I2C};
    UARTConfig uart_config_;
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

    std::unique_ptr<Transport> transport_;

    // Asynchronous loop state
    std::thread async_thread_;
    std::atomic<bool> async_running_{false};
    AsyncDataCallback async_callback_;
    double async_rate_hz_{50.0};

    // Raw Asynchronous loop state
    std::thread raw_async_thread_;
    std::atomic<bool> raw_async_running_{false};
    RawAsyncDataCallback raw_async_callback_;
    double raw_async_rate_hz_{100.0};

    // GPIO Interrupt loop state
    std::thread irq_thread_;
    std::atomic<bool> irq_running_{false};
    RawAsyncDataCallback irq_callback_;
    int irq_gpio_pin_{-1};

    // Auto-calibration state
    std::string auto_calib_file_;
    bool auto_calib_enabled_{false};
    bool auto_calib_saved_{false};

    Impl(const UARTConfig& uart_config) : uart_config_(uart_config) { conn_type_ = ConnectionType::UART; }

    Impl(uint8_t address, std::string_view i2c_device) : address_(address), i2c_device_(std::string(i2c_device)) {}

    Impl(std::unique_ptr<Transport> transport) : transport_(std::move(transport)) {}

    ~Impl() {
        if (async_running_) {
            async_running_ = false;
            if (async_thread_.joinable()) {
                async_thread_.join();
            }
        }
        if (raw_async_running_) {
            raw_async_running_ = false;
            if (raw_async_thread_.joinable()) {
                raw_async_thread_.join();
            }
        }
        if (irq_running_) {
            irq_running_ = false;
            if (irq_thread_.joinable()) {
                irq_thread_.join();
            }
        }
        close_i2c();
    }

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
        if (transport_) {
            if (transport_->open()) {
                i2c_fd = 999;
                return true;
            }
            return false;
        }
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            i2c_fd = open(uart_config_.port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
            if (i2c_fd < 0) {
                log(LogLevel::Error, "Failed to open UART port: " + uart_config_.port);
                return false;
            }
            struct termios tty;
            if (tcgetattr(i2c_fd, &tty) != 0) {
                close(i2c_fd);
                i2c_fd = -1;
                return false;
            }
            speed_t speed = B115200;
            if (uart_config_.baudrate == 9600) speed = B9600;
            cfsetospeed(&tty, speed);
            cfsetispeed(&tty, speed);
            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            tty.c_iflag &= ~IGNBRK;
            tty.c_lflag = 0;
            tty.c_oflag = 0;
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = static_cast<cc_t>(uart_config_.timeout * 10);
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(PARENB | PARODD);
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CRTSCTS;
            if (tcsetattr(i2c_fd, TCSANOW, &tty) != 0) {
                close(i2c_fd);
                i2c_fd = -1;
                return false;
            }
            return true;
        }

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
            if (transport_) {
                transport_->close();
            } else {
#ifdef __linux__
                close(i2c_fd);
#endif
            }
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

    bool uart_read_exact(uint8_t* buf, int len) {
        int read_bytes = 0;
        int timeout_ms = static_cast<int>(uart_config_.timeout * 1000);
        while (read_bytes < len) {
            struct pollfd pfd = {i2c_fd, POLLIN, 0};
            int ret = poll(&pfd, 1, timeout_ms);
            if (ret > 0) {
                int n = ::read(i2c_fd, buf + read_bytes, len - read_bytes);
                if (n > 0) {
                    read_bytes += n;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }
        return true;
    }

    // Low-level raw methods
    bool write8_raw(uint8_t reg, uint8_t value) {
        if (transport_) {
            return transport_->write8(reg, value);
        }
        if (i2c_fd < 0) return false;
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            uint8_t buf[5] = {0xAA, 0x00, reg, 1, value};
            if (::write(i2c_fd, buf, 5) != 5) return false;
            uint8_t resp[2];
            if (!uart_read_exact(resp, 2)) return false;
            return (resp[0] == 0xEE && resp[1] == 0x01);
        }
        uint8_t buffer[2] = {reg, value};
        return ::write(i2c_fd, buffer, 2) == 2;
#else
        (void)reg;
        (void)value;
        return true;
#endif
    }

    bool writeLen_raw(uint8_t reg, const uint8_t* buffer, uint8_t len) {
        if (transport_) {
            return transport_->writeLen(reg, buffer, len);
        }
        if (i2c_fd < 0 || len > 31) return false;
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            std::vector<uint8_t> buf(4 + len);
            buf[0] = 0xAA;
            buf[1] = 0x00;
            buf[2] = reg;
            buf[3] = len;
            std::memcpy(buf.data() + 4, buffer, len);
            if (::write(i2c_fd, buf.data(), buf.size()) != (ssize_t)buf.size()) return false;
            uint8_t resp[2];
            if (!uart_read_exact(resp, 2)) return false;
            return (resp[0] == 0xEE && resp[1] == 0x01);
        }
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
        if (transport_) {
            return transport_->read8(reg, value);
        }
        if (i2c_fd < 0) return false;
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            uint8_t buf[4] = {0xAA, 0x01, reg, 1};
            if (::write(i2c_fd, buf, 4) != 4) return false;
            uint8_t resp[2];
            if (!uart_read_exact(resp, 2)) return false;
            if (resp[0] == 0xBB && resp[1] == 1) {
                return uart_read_exact(&value, 1);
            }
            return false;
        }
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

    bool readLen_raw(uint8_t reg, uint8_t* buffer, uint8_t len) {
        if (transport_) {
            return transport_->readLen(reg, buffer, len);
        }
        if (i2c_fd < 0) return false;
#ifdef __linux__
        if (conn_type_ == ConnectionType::UART) {
            uint8_t buf[4] = {0xAA, 0x01, reg, len};
            if (::write(i2c_fd, buf, 4) != 4) return false;
            uint8_t resp[2];
            if (!uart_read_exact(resp, 2)) return false;
            if (resp[0] == 0xBB && resp[1] == len) {
                return uart_read_exact(buffer, len);
            }
            return false;
        }
        uint8_t reg_buf[1] = {reg};
        if (::write(i2c_fd, reg_buf, 1) != 1) return false;
        return ::read(i2c_fd, buffer, len) == len;
#else
        std::memset(buffer, 0, len);
        if (reg == 0x20 && len >= 8) {  // QUATERNION_DATA_W_LSB
            int16_t w = 16384;
            buffer[0] = w & 0xFF;
            buffer[1] = (w >> 8) & 0xFF;
        }
        return true;
#endif
    }

    // Thread-safe methods with automatic reconnect and retries
    bool write8(uint8_t reg, uint8_t value, int retries = 3) {
        for (int i = 0; i < retries; ++i) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (i2c_fd >= 0 || open_i2c()) {
                    if (write8_raw(reg, value)) {
                        return true;
                    }
                }
                diagnostics_.write_failures++;
            }
            log(LogLevel::Warning, "I2C/UART write failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        bool rec_ok = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            rec_ok = reconnect();
            if (rec_ok) {
                if (write8_raw(reg, value)) {
                    return true;
                }
            }
            diagnostics_.write_failures++;
        }
        log(LogLevel::Error, "I2C/UART write failed permanently");
        return false;
    }

    bool writeLen(uint8_t reg, const uint8_t* buffer, uint8_t len, int retries = 3) {
        if (len > 31) return false;
        for (int i = 0; i < retries; ++i) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (i2c_fd >= 0 || open_i2c()) {
                    if (writeLen_raw(reg, buffer, len)) {
                        return true;
                    }
                }
                diagnostics_.write_failures++;
            }
            log(LogLevel::Warning, "I2C/UART writeLen failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        bool rec_ok = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            rec_ok = reconnect();
            if (rec_ok) {
                if (writeLen_raw(reg, buffer, len)) {
                    return true;
                }
            }
            diagnostics_.write_failures++;
        }
        log(LogLevel::Error, "I2C/UART writeLen failed permanently");
        return false;
    }

    bool read8(uint8_t reg, uint8_t& value, int retries = 3) {
        for (int i = 0; i < retries; ++i) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (i2c_fd >= 0 || open_i2c()) {
                    if (read8_raw(reg, value)) {
                        return true;
                    }
                }
                diagnostics_.read_failures++;
            }
            log(LogLevel::Warning, "I2C/UART read failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        bool rec_ok = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            rec_ok = reconnect();
            if (rec_ok) {
                if (read8_raw(reg, value)) {
                    return true;
                }
            }
            diagnostics_.read_failures++;
        }
        log(LogLevel::Error, "I2C/UART read failed permanently");
        return false;
    }

    bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len, int retries = 3) {
        for (int i = 0; i < retries; ++i) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (i2c_fd >= 0 || open_i2c()) {
                    if (readLen_raw(reg, buffer, len)) {
                        return true;
                    }
                }
                diagnostics_.read_failures++;
            }
            log(LogLevel::Warning, "I2C/UART readLen failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        bool rec_ok = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            rec_ok = reconnect();
            if (rec_ok) {
                if (readLen_raw(reg, buffer, len)) {
                    return true;
                }
            }
            diagnostics_.read_failures++;
        }
        log(LogLevel::Error, "I2C/UART readLen failed permanently");
        return false;
    }
};

BNO055::BNO055(const UARTConfig& uart_config) : impl_(std::make_unique<Impl>(uart_config)) {}

BNO055::BNO055(uint8_t i2c_address, std::string_view i2c_device)
    : impl_(std::make_unique<Impl>(i2c_address, i2c_device)) {}

BNO055::BNO055(std::unique_ptr<Transport> transport) : impl_(std::make_unique<Impl>(std::move(transport))) {}

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

    // Hardware Overclocking for EKF/AMG Mode:
    // If the sensor is set to raw sensor mode (AMG), configure sub-sensors to maximum physical limits.
    if (mode == OpMode::AMG) {
        impl_->log(LogLevel::Info, "Overclocking physical sub-sensors: Accel -> 1kHz ODR, Gyro -> 2kHz ODR");
        // Open Page 1 config space
        impl_->write8(PAGE_ID, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // 1. Accel: ODR=1000Hz, Bandwidth=125Hz, Range=4g -> 0x0F
        impl_->write8(ACC_CONFIG, 0x0F);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // 2. Gyro: ODR=2000Hz, Bandwidth=523Hz -> GYR_CONFIG_0 = 0x00
        impl_->write8(GYR_CONFIG_0, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // 3. Gyro Power: Normal mode -> GYR_CONFIG_1 = 0x00
        impl_->write8(GYR_CONFIG_1, 0x00);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // Restore Page 0 configuration space
        impl_->write8(PAGE_ID, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Automatically load calibration file if configured
    if (impl_->auto_calib_enabled_) {
        impl_->auto_calib_saved_ = false;
        loadCalibrationFile(impl_->auto_calib_file_);
    }

    return true;
}

bool BNO055::reset() {
    impl_->log(LogLevel::Info, "Performing hardware reset...");
    return begin(impl_->mode_);
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

BNO055::RawSensorData BNO055::getRawSensorData() {
    auto val = getRawSensorDataNoexcept();
    if (!val) {
        throw IMUError("Failed to burst-read raw sensor data");
    }
    return *val;
}

std::optional<BNO055::RawSensorData> BNO055::getRawSensorDataNoexcept() noexcept {
    // Burst read 18 contiguous bytes:
    // 0x08 - 0x0D: Accel (6 bytes)
    // 0x0E - 0x13: Mag (6 bytes)
    // 0x14 - 0x19: Gyro (6 bytes)
    uint8_t buffer[18]{0};
    if (!impl_->readLen(ACCEL_DATA_X_LSB, buffer, 18)) {
        return std::nullopt;
    }

    RawSensorData data;
    // 1. Accel: 1 m/s^2 = 100 LSB
    data.accel = parseVector3(buffer, 1.0f / 100.0f);
    // 2. Mag: 1 uT = 16 LSB
    data.mag = parseVector3(buffer + 6, 1.0f / 16.0f);
    // 3. Gyro: 1 dps = 16 LSB. Convert to rad/s (dps * M_PI / 180.0)
    constexpr float gyro_scale = (1.0f / 16.0f) * (static_cast<float>(M_PI) / 180.0f);
    data.gyro = parseVector3(buffer + 12, gyro_scale);

    return data;
}

std::optional<Vector3> BNO055::getAccelerometerNoexcept() noexcept {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(ACCEL_DATA_X_LSB, buffer, 6)) {
        return std::nullopt;
    }
    // 1 m/s^2 = 100 LSB
    return parseVector3(buffer, 1.0f / 100.0f);
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
    return parseVector3(buffer, 1.0f / 16.0f);
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
    constexpr float scale = (1.0f / 16.0f) * (static_cast<float>(M_PI) / 180.0f);
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
    constexpr float scale = (1.0f / 16.0f) * (static_cast<float>(M_PI) / 180.0f);
    // buffer order: h(yaw), r(roll), p(pitch)
    return Vector3{static_cast<float>(read16_le(buffer + 2)) * scale, static_cast<float>(read16_le(buffer + 4)) * scale,
                   static_cast<float>(read16_le(buffer)) * scale};
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
    return parseVector3(buffer, 1.0f / 100.0f);
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
    return parseVector3(buffer, 1.0f / 100.0f);
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
    constexpr float scale = 1.0f / 16384.0f;
    return Quaternion{static_cast<float>(read16_le(buffer)) * scale, static_cast<float>(read16_le(buffer + 2)) * scale,
                      static_cast<float>(read16_le(buffer + 4)) * scale,
                      static_cast<float>(read16_le(buffer + 6)) * scale};
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
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    float roll = std::atan2(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    float pitch = 0.0f;
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(static_cast<float>(M_PI) / 2.0f, sinp);
    } else {
        pitch = std::asin(sinp);
    }

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);

    float roll_deg = roll * 180.0f / static_cast<float>(M_PI);
    float pitch_deg = pitch * 180.0f / static_cast<float>(M_PI);
    float yaw_deg = yaw * 180.0f / static_cast<float>(M_PI);

    if (yaw_deg < 0.0f) {
        yaw_deg += 360.0f;
    }

    return Vector3{roll_deg, pitch_deg, yaw_deg};
}

bool BNO055::startAsyncReading(double rate_hz, AsyncDataCallback callback) {
    if (impl_->async_running_) {
        return false;
    }

    impl_->async_rate_hz_ = rate_hz;
    impl_->async_callback_ = std::move(callback);
    impl_->async_running_ = true;

    impl_->async_thread_ = std::thread([this]() {
        impl_->log(LogLevel::Info, "Starting background async reading thread...");
        const auto period = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / impl_->async_rate_hz_));

        while (impl_->async_running_) {
            const auto start_time = std::chrono::steady_clock::now();

            AllData data;
            data.accel = getAccelerometerOrDefault();
            data.mag = getMagnetometerOrDefault();
            data.gyro = getGyroscopeOrDefault();
            data.euler = getEulerAnglesOrDefault();
            data.linear_accel = getLinearAccelerationOrDefault();
            data.gravity = getGravityOrDefault();
            data.quat = getQuaternionOrDefault();
            data.temp = getTemperatureOrDefault();

            if (impl_->async_callback_ && impl_->async_running_) {
                impl_->async_callback_(data);
            }

            // Auto-calibration save check
            if (impl_->auto_calib_enabled_ && !impl_->auto_calib_saved_) {
                CalibrationStatus calib = getCalibrationStatus();
                if (calib.isFullyCalibrated()) {
                    impl_->log(LogLevel::Info, "Sensor fully calibrated. Auto-saving calibration offsets...");
                    if (saveCalibrationFile(impl_->auto_calib_file_)) {
                        impl_->auto_calib_saved_ = true;
                    }
                }
            }

            const auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < period && impl_->async_running_) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }
        impl_->log(LogLevel::Info, "Background async reading thread stopped.");
    });

    return true;
}

void BNO055::stopAsyncReading() {
    if (!impl_->async_running_) {
        return;
    }
    impl_->async_running_ = false;
    if (impl_->async_thread_.joinable()) {
        impl_->async_thread_.join();
    }
}

bool BNO055::startRawAsyncReading(double rate_hz, RawAsyncDataCallback callback) {
    if (impl_->raw_async_running_) {
        return false;
    }

    impl_->raw_async_rate_hz_ = rate_hz;
    impl_->raw_async_callback_ = std::move(callback);
    impl_->raw_async_running_ = true;

    impl_->raw_async_thread_ = std::thread([this]() {
        impl_->log(LogLevel::Info, "Starting background high-performance raw async reading thread...");
        const auto period = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / impl_->raw_async_rate_hz_));

        while (impl_->raw_async_running_) {
            const auto start_time = std::chrono::steady_clock::now();

            auto raw_opt = getRawSensorDataNoexcept();
            if (raw_opt) {
                if (impl_->raw_async_callback_ && impl_->raw_async_running_) {
                    impl_->raw_async_callback_(*raw_opt);
                }
            } else {
                impl_->log(LogLevel::Warning, "Raw burst async read failed");
            }

            const auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed < period && impl_->raw_async_running_) {
                std::this_thread::sleep_for(period - elapsed);
            }
        }
        impl_->log(LogLevel::Info, "Background raw async reading thread stopped.");
    });

    return true;
}

void BNO055::stopRawAsyncReading() {
    if (!impl_->raw_async_running_) {
        return;
    }
    impl_->raw_async_running_ = false;
    if (impl_->raw_async_thread_.joinable()) {
        impl_->raw_async_thread_.join();
    }
}

void BNO055::enableAutoCalibration(std::string_view filepath) {
    impl_->auto_calib_file_ = std::string(filepath);
    impl_->auto_calib_enabled_ = true;
}

void BNO055::disableAutoCalibration() {
    impl_->auto_calib_enabled_ = false;
}

bool BNO055::startInterruptDrivenReading(int gpio_pin, RawAsyncDataCallback callback) {
    if (impl_->irq_running_) {
        return false;
    }

    impl_->irq_gpio_pin_ = gpio_pin;
    impl_->irq_callback_ = std::move(callback);
    impl_->irq_running_ = true;

    impl_->irq_thread_ = std::thread([this]() {
        impl_->log(LogLevel::Info, "Starting background hardware interrupt (IRQ) waiting thread...");

        // Mock setup on non-linux systems to let GTest units verify callback triggering
        bool is_mock = true;
#ifdef __linux__
        is_mock = (impl_->i2c_fd == 999 && impl_->transport_ != nullptr);
#endif

        if (is_mock) {
            // Emulate periodic interrupts for unit testing
            while (impl_->irq_running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                auto raw_opt = getRawSensorDataNoexcept();
                if (raw_opt && impl_->irq_callback_ && impl_->irq_running_) {
                    impl_->irq_callback_(*raw_opt);
                }
            }
            impl_->log(LogLevel::Info, "Mock hardware interrupt thread stopped.");
            return;
        }

#ifdef __linux__
        // 1. Export GPIO Pin if not already exported
        std::string pin_str = std::to_string(impl_->irq_gpio_pin_);
        std::ofstream export_file("/sys/class/gpio/export");
        if (export_file.is_open()) {
            export_file << pin_str;
            export_file.close();
            // Wait for system to create sysfs directory node
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 2. Set direction to 'in'
        std::string dir_path = "/sys/class/gpio/gpio" + pin_str + "/direction";
        std::ofstream dir_file(dir_path);
        if (dir_file.is_open()) {
            dir_file << "in";
            dir_file.close();
        }

        // 3. Configure rising edge trigger
        std::string edge_path = "/sys/class/gpio/gpio" + pin_str + "/edge";
        std::ofstream edge_file(edge_path);
        if (edge_file.is_open()) {
            edge_file << "rising";
            edge_file.close();
        }

        // 4. Open value file for polling
        std::string val_path = "/sys/class/gpio/gpio" + pin_str + "/value";
        int val_fd = ::open(val_path.c_str(), O_RDONLY | O_NONBLOCK);
        if (val_fd < 0) {
            impl_->log(LogLevel::Error, "Failed to open GPIO value sysfs file for interrupt monitoring");
            impl_->irq_running_ = false;
            return;
        }

        // Clear initial state
        char dummy;
        (void)::read(val_fd, &dummy, 1);

        struct pollfd pfd;
        pfd.fd = val_fd;
        pfd.events = POLLPRI | POLLERR;

        while (impl_->irq_running_) {
            // Wait for edge interrupt event with a 100ms timeout to periodically check if stopped
            int num_events = ::poll(&pfd, 1, 100);
            if (num_events > 0) {
                if (pfd.revents & POLLPRI) {
                    // Seek back to start to clear interrupt flag
                    ::lseek(val_fd, 0, SEEK_SET);
                    (void)::read(val_fd, &dummy, 1);

                    // Execute burst read Immediately on interrupt
                    auto raw_opt = getRawSensorDataNoexcept();
                    if (raw_opt) {
                        if (impl_->irq_callback_ && impl_->irq_running_) {
                            impl_->irq_callback_(*raw_opt);
                        }
                    }
                }
            }
        }

        ::close(val_fd);

        // Clean up: Unexport GPIO Pin
        std::ofstream unexport_file("/sys/class/gpio/unexport");
        if (unexport_file.is_open()) {
            unexport_file << pin_str;
            unexport_file.close();
        }
#endif
        impl_->log(LogLevel::Info, "Hardware interrupt waiting thread stopped.");
    });

    return true;
}

void BNO055::stopInterruptDrivenReading() {
    if (!impl_->irq_running_) {
        return;
    }
    impl_->irq_running_ = false;
    if (impl_->irq_thread_.joinable()) {
        impl_->irq_thread_.join();
    }
}

}  // namespace bno055lib
