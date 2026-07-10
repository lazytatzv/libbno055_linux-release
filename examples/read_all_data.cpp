#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "libbno055-linux/bno055.hpp"

// Helper function to print a simple ASCII visual bar indicator for orientation angle [-90, 90]
void printBarIndicator(const std::string& label, double val_deg) {
    const int width = 20;
    const int center = width / 2;

    // Clamp to range
    if (val_deg < -90.0) val_deg = -90.0;
    if (val_deg > 90.0) val_deg = 90.0;

    int pos = center + static_cast<int>((val_deg / 90.0) * center);
    if (pos < 0) pos = 0;
    if (pos >= width) pos = width - 1;

    std::string bar(width, ' ');
    bar[center] = '|';
    bar[pos] = '*';

    std::cout << label << " [" << bar << "] " << std::fixed << std::setprecision(2) << std::setw(7) << val_deg << " deg"
              << std::endl;
}

int main(int argc, char* argv[]) {
    std::string device = "/dev/i2c-1";
    bno055lib::OpMode mode = bno055lib::OpMode::NDOF;
    std::string mode_str = "ndof";

    if (argc > 1) {
        device = argv[1];
    }
    if (argc > 2) {
        mode_str = argv[2];
        if (mode_str == "imu") {
            mode = bno055lib::OpMode::IMUPlus;
        } else if (mode_str == "amg") {
            mode = bno055lib::OpMode::AMG;
        } else if (mode_str == "gyro") {
            mode = bno055lib::OpMode::GyroOnly;
        } else if (mode_str == "ndof") {
            mode = bno055lib::OpMode::NDOF;
        } else {
            std::cerr << "Unknown mode: " << mode_str << ". Defaulting to ndof." << std::endl;
            mode_str = "ndof";
        }
    }

    std::cout << "Initializing BNO055 IMU on " << device << " (Mode: " << mode_str << ")..." << std::endl;
    bno055lib::BNO055 imu(0x28, device);

    // Use default logger (standard error output)
    if (!imu.begin(mode)) {
        std::cerr << "Failed to initialize BNO055!" << std::endl;
        return 1;
    }

    std::cout << "BNO055 initialized. Displaying all sensor readings (10Hz)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while (true) {
        try {
            // Read all physical data types provided by BNO055
            auto accel = imu.getAccelerometer();
            auto mag = imu.getMagnetometer();
            auto gyro = imu.getGyroscope();
            auto euler = imu.getEulerAngles();
            auto linear = imu.getLinearAcceleration();
            auto gravity = imu.getGravity();
            auto quat = imu.getQuaternion();
            auto temp = imu.getTemperature();
            auto calib = imu.getCalibrationStatus();

            // Convert quaternion to euler degrees for human visualization
            auto euler_deg = bno055lib::toEulerDegrees(quat);

            // Clear console using ANSI escape codes for clean terminal refresh
            std::cout << "\033[2J\033[H";
            std::cout << "================== BNO055 Visual Dashboard ==================" << std::endl;
            std::cout << "Device: " << device << "  |  Mode: " << mode_str << std::endl;
            std::cout << "Calibration: SYS=" << (int)calib.sys << " GYRO=" << (int)calib.gyro
                      << " ACCEL=" << (int)calib.accel << " MAG=" << (int)calib.mag << std::endl;
            std::cout << "-------------------------------------------------------------" << std::endl;

            // Visual indicators for tilt (Roll and Pitch)
            printBarIndicator("Roll  (X)", euler_deg.x);
            printBarIndicator("Pitch (Y)", euler_deg.y);
            std::cout << "Yaw   (Z)  " << std::fixed << std::setprecision(2) << std::setw(7) << euler_deg.z << " deg"
                      << std::endl;
            std::cout << "-------------------------------------------------------------" << std::endl;

            std::cout << "Accelerometer (m/s^2):       X=" << std::setw(7) << accel.x << " Y=" << std::setw(7)
                      << accel.y << " Z=" << std::setw(7) << accel.z << std::endl;
            std::cout << "Magnetometer (uT):           X=" << std::setw(7) << mag.x << " Y=" << std::setw(7) << mag.y
                      << " Z=" << std::setw(7) << mag.z << std::endl;
            std::cout << "Gyroscope (rad/s):           X=" << std::setw(7) << gyro.x << " Y=" << std::setw(7) << gyro.y
                      << " Z=" << std::setw(7) << gyro.z << std::endl;
            std::cout << "Euler Angles (rad):          Roll=" << std::setw(7) << euler.x << " Pitch=" << std::setw(7)
                      << euler.y << " Yaw=" << std::setw(7) << euler.z << std::endl;
            std::cout << "Linear Acceleration (m/s^2):  X=" << std::setw(7) << linear.x << " Y=" << std::setw(7)
                      << linear.y << " Z=" << std::setw(7) << linear.z << std::endl;
            std::cout << "Gravity Vector (m/s^2):       X=" << std::setw(7) << gravity.x << " Y=" << std::setw(7)
                      << gravity.y << " Z=" << std::setw(7) << gravity.z << std::endl;
            std::cout << "Quaternion:                  W=" << std::setw(7) << quat.w << " X=" << std::setw(7) << quat.x
                      << " Y=" << std::setw(7) << quat.y << " Z=" << std::setw(7) << quat.z << std::endl;
            std::cout << "Temperature (C):             " << (int)temp << std::endl;

        } catch (const bno055lib::IMUError& e) {
            std::cout << "\033[2J\033[H";
            std::cerr << "Communication error: " << e.what() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz
    }

    return 0;
}
