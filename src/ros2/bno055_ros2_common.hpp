#ifndef BNO055_ROS2_COMMON_HPP
#define BNO055_ROS2_COMMON_HPP

#include <algorithm>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <vector>

#include "libbno055-linux/bno055.hpp"

namespace bno055_ros2 {

// Declare standard parameters
inline void declare_common_parameters(rclcpp::Node* node) {
    node->declare_parameter<std::string>("device", "/dev/i2c-1");
    node->declare_parameter<int>("address", 0x28);
    node->declare_parameter<std::string>("frame_id", "imu_link");
    node->declare_parameter<double>("publish_rate", 50.0);
    node->declare_parameter<std::string>("qos_reliability", "best_effort");
    node->declare_parameter<int>("qos_history_depth", 10);
    node->declare_parameter<std::string>("calibration_file", "");
    node->declare_parameter<std::vector<double>>("orientation_covariance", std::vector<double>(9, 0.0));
    node->declare_parameter<std::vector<double>>("angular_velocity_covariance", std::vector<double>(9, 0.0));
    node->declare_parameter<std::vector<double>>("linear_acceleration_covariance", std::vector<double>(9, 0.0));
}

// Redirect logger callback
inline void setup_logger_redirection(rclcpp::Node* node, bno055lib::BNO055& imu) {
    imu.setLogger([node](bno055lib::LogLevel level, std::string_view message) {
        switch (level) {
            case bno055lib::LogLevel::Debug:
                RCLCPP_DEBUG(node->get_logger(), "%s", message.data());
                break;
            case bno055lib::LogLevel::Info:
                RCLCPP_INFO(node->get_logger(), "%s", message.data());
                break;
            case bno055lib::LogLevel::Warning:
                RCLCPP_WARN(node->get_logger(), "%s", message.data());
                break;
            case bno055lib::LogLevel::Error:
                RCLCPP_ERROR(node->get_logger(), "%s", message.data());
                break;
        }
    });
}

// Set covariances from parameters
inline void fill_imu_covariances(rclcpp::Node* node, sensor_msgs::msg::Imu& message) {
    auto ori_cov = node->get_parameter("orientation_covariance").as_double_array();
    auto gyro_cov = node->get_parameter("angular_velocity_covariance").as_double_array();
    auto accel_cov = node->get_parameter("linear_acceleration_covariance").as_double_array();

    if (ori_cov.size() == 9) std::copy(ori_cov.begin(), ori_cov.end(), message.orientation_covariance.begin());
    if (gyro_cov.size() == 9) std::copy(gyro_cov.begin(), gyro_cov.end(), message.angular_velocity_covariance.begin());
    if (accel_cov.size() == 9)
        std::copy(accel_cov.begin(), accel_cov.end(), message.linear_acceleration_covariance.begin());
}

// Build standard diagnostic array
inline diagnostic_msgs::msg::DiagnosticArray::UniquePtr build_diagnostics(rclcpp::Node* node, bno055lib::BNO055& imu,
                                                                          const std::string& node_name) {
    auto diag_arr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
    diag_arr->header.stamp = node->now();

    auto status = diagnostic_msgs::msg::DiagnosticStatus();
    status.name = std::string("libbno055_linux: ") + node_name;
    status.hardware_id =
        node->get_parameter("device").as_string() + ":" + std::to_string(node->get_parameter("address").as_int());

    auto diag = imu.getDiagnostics();

    auto add_key_value = [](diagnostic_msgs::msg::DiagnosticStatus& stat, const std::string& key,
                            const std::string& val) {
        auto kv = diagnostic_msgs::msg::KeyValue();
        kv.key = key;
        kv.value = val;
        stat.values.push_back(kv);
    };

    add_key_value(status, "I2C Read Failures", std::to_string(diag.read_failures));
    add_key_value(status, "I2C Write Failures", std::to_string(diag.write_failures));
    add_key_value(status, "I2C Reconnect Attempts", std::to_string(diag.reconnect_attempts));

    bno055lib::CalibrationStatus calib;
    bool calib_ok = false;
    try {
        calib = imu.getCalibrationStatus();
        calib_ok = true;
    } catch (const std::exception& e) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = std::string("Failed to query calibration status: ") + e.what();
    }

    if (calib_ok) {
        add_key_value(status, "Calibration Status: Sys", std::to_string(calib.sys));
        add_key_value(status, "Calibration Status: Gyro", std::to_string(calib.gyro));
        add_key_value(status, "Calibration Status: Accel", std::to_string(calib.accel));
        add_key_value(status, "Calibration Status: Mag", std::to_string(calib.mag));

        if (calib.isFullyCalibrated()) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Fully Calibrated & Streaming";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Calibration incomplete (Sys:" + std::to_string(calib.sys) +
                             " G:" + std::to_string(calib.gyro) + " A:" + std::to_string(calib.accel) +
                             " M:" + std::to_string(calib.mag) + ")";
        }
    }

    if (diag.reconnect_attempts > 5) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "Unstable I2C connection. Reconnected " + std::to_string(diag.reconnect_attempts) + " times.";
    }

    diag_arr->status.push_back(status);
    return diag_arr;
}

}  // namespace bno055_ros2

#endif  // BNO055_ROS2_COMMON_HPP
