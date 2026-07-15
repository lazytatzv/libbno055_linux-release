#include <gtest/gtest.h>

#include "libbno055-linux/bno055.hpp"

TEST(BNO055Test, ConstructorAndMockInitialization) {
#ifndef __linux__
    // On macOS/Windows, the Mock mode should allow begin() to succeed.
    bno055lib::BNO055 imu(0x28, "/dev/i2c-mock");
    EXPECT_TRUE(imu.begin(bno055lib::OpMode::NDOF));

    // Check initial diagnostics (should be 0)
    auto diag = imu.getDiagnostics();
    EXPECT_EQ(diag.write_failures, 0u);
    EXPECT_EQ(diag.read_failures, 0u);
    EXPECT_EQ(diag.reconnect_attempts, 0u);

    // Check Mock quaternion data (should return w=1.0)
    auto quat_opt = imu.getQuaternionNoexcept();
    ASSERT_TRUE(quat_opt.has_value());
    EXPECT_NEAR(quat_opt->w, 1.0, 1e-4);
    EXPECT_NEAR(quat_opt->x, 0.0, 1e-4);
    EXPECT_NEAR(quat_opt->y, 0.0, 1e-4);
    EXPECT_NEAR(quat_opt->z, 0.0, 1e-4);
#else
    // On Linux, if mock is not active, begin() with non-existent device should fail
    bno055lib::BNO055 imu(0x28, "/dev/i2c-invalid-device-node");
    EXPECT_FALSE(imu.begin(bno055lib::OpMode::NDOF));
#endif
}

TEST(BNO055Test, CalibrationStatusDefault) {
#ifndef __linux__
    bno055lib::BNO055 imu(0x28, "/dev/i2c-mock");
    ASSERT_TRUE(imu.begin(bno055lib::OpMode::NDOF));

    // In mock mode, raw reads return 0, so calibration values should be 0
    auto calib = imu.getCalibrationStatus();
    EXPECT_EQ(calib.sys, 0);
    EXPECT_EQ(calib.gyro, 0);
    EXPECT_EQ(calib.accel, 0);
    EXPECT_EQ(calib.mag, 0);
    EXPECT_FALSE(calib.isFullyCalibrated());
#endif
}

TEST(BNO055Test, NoexceptGettersAndEulerConversion) {
#ifndef __linux__
    bno055lib::BNO055 imu(0x28, "/dev/i2c-mock");
    ASSERT_TRUE(imu.begin(bno055lib::OpMode::NDOF));

    // Check Noexcept values in Mock mode
    auto accel_opt = imu.getAccelerometerNoexcept();
    ASSERT_TRUE(accel_opt.has_value());
    EXPECT_NEAR(accel_opt->x, 0.0, 1e-4);
    EXPECT_NEAR(accel_opt->y, 0.0, 1e-4);
    EXPECT_NEAR(accel_opt->z, 0.0, 1e-4);

    auto quat_opt = imu.getQuaternionNoexcept();
    ASSERT_TRUE(quat_opt.has_value());
    EXPECT_NEAR(quat_opt->w, 1.0, 1e-4);
    EXPECT_NEAR(quat_opt->x, 0.0, 1e-4);
    EXPECT_NEAR(quat_opt->y, 0.0, 1e-4);
    EXPECT_NEAR(quat_opt->z, 0.0, 1e-4);

    // Verify Quaternion-to-Euler conversion utility
    // Identity quaternion should result in Roll=0, Pitch=0, Yaw=0
    auto euler = bno055lib::toEulerDegrees(*quat_opt);
    EXPECT_NEAR(euler.x, 0.0, 1e-4);  // Roll
    EXPECT_NEAR(euler.y, 0.0, 1e-4);  // Pitch
    EXPECT_NEAR(euler.z, 0.0, 1e-4);  // Yaw

    // Rotate 90 degrees around Yaw (Z-axis)
    // q = cos(45deg) + sin(45deg)*z = 0.7071068 + 0.7071068*z
    bno055lib::Quaternion q_rot;
    q_rot.w = std::cos(M_PI / 4.0);
    q_rot.x = 0.0;
    q_rot.y = 0.0;
    q_rot.z = std::sin(M_PI / 4.0);

    auto euler_rot = bno055lib::toEulerDegrees(q_rot);
    EXPECT_NEAR(euler_rot.x, 0.0, 1e-4);
    EXPECT_NEAR(euler_rot.y, 0.0, 1e-4);
    EXPECT_NEAR(euler_rot.z, 90.0, 1e-4);  // Should be exactly 90 degrees
#endif
}
