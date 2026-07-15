/**
 * @file ros2_perf_publisher_node.cpp
 * @brief High-performance ROS 2 publisher node optimized for low-latency and zero-copy transport.
 *
 * This node is tailored for resource-constrained embedded Linux systems.
 * It uses ROS 2 intra-process communication features and passes message pointers
 * via std::unique_ptr and std::move. In a single-process composable container,
 * this design completely bypasses message serialization and copy overhead.
 * It also uses exception-free (noexcept) APIs to maintain deterministic cycle times.
 */

#include <algorithm>
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <utility>

#include "bno055_ros2_common.hpp"
#include "libbno055-linux/bno055.hpp"

using namespace std::chrono_literals;

namespace bno055_ros2 {

class BNO055PerfPublisherNode : public rclcpp::Node {
public:
    explicit BNO055PerfPublisherNode(const rclcpp::NodeOptions& options)
        : Node("bno055_perf_publisher_node", options), imu_(0x28, "/dev/i2c-1") {
        // Declare and fetch parameters using common helpers
        bno055_ros2::declare_common_parameters(this);

        std::string device = this->get_parameter("device").as_string();
        int address = this->get_parameter("address").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double rate_hz = this->get_parameter("publish_rate").as_double();
        std::string calib_file = this->get_parameter("calibration_file").as_string();

        RCLCPP_INFO(this->get_logger(), "Initializing High-Performance BNO055 Node on %s (address: 0x%02X)",
                    device.c_str(), address);

        imu_ = bno055lib::BNO055(address, device);

        // Redirect internal logs into ROS 2 RCLCPP
        bno055_ros2::setup_logger_redirection(this, imu_);

        // Use high-performance NDOF mode (includes sensor fusion)
        auto mode = bno055_ros2::parse_op_mode(this->get_parameter("operation_mode").as_string());
        if (!imu_.begin(mode)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize BNO055 sensor!");
            throw std::runtime_error("Sensor initialization failed");
        }

        // Apply advanced hardware configurations
        bno055_ros2::apply_advanced_features(this, imu_);

        // Load calibration file if specified
        if (!calib_file.empty()) {
            if (imu_.loadCalibrationFile(calib_file)) {
                RCLCPP_INFO(this->get_logger(), "Loaded calibration offsets from: %s", calib_file.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to load calibration file: %s", calib_file.c_str());
            }
        }

        // Setup dynamic QoS
        auto qos = rclcpp::SensorDataQoS();
        std::string reliability = this->get_parameter("qos_reliability").as_string();
        if (reliability == "reliable") {
            qos.reliable();
        } else if (reliability == "best_effort") {
            qos.best_effort();
        }
        qos.keep_last(this->get_parameter("qos_history_depth").as_int());

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
        mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);
        temp_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temp", qos);
        save_calib_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/save_calibration", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                std::string cf = this->get_parameter("calibration_file").as_string();
                if (cf.empty()) {
                    response->success = false;
                    response->message = "calibration_file parameter is empty.";
                    return;
                }
                if (imu_.saveCalibrationFile(cf)) {
                    response->success = true;
                    response->message = "Successfully saved calibration to " + cf;
                } else {
                    response->success = false;
                    response->message = "Failed to save calibration file.";
                }
            });

        diag_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        auto interval = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(interval, std::bind(&BNO055PerfPublisherNode::timer_callback, this));
        diag_timer_ = this->create_wall_timer(1s, std::bind(&BNO055PerfPublisherNode::publish_diagnostics, this));

        RCLCPP_INFO(this->get_logger(), "IMU Perf Node started. Publishing on 'imu/data' at %.1f Hz", rate_hz);
    }

private:
    void timer_callback() {
        // Record timestamp immediately before I2C communication starts to minimize jitter
        auto stamp = this->now();

        // High-performance exception-free (noexcept) reads
        auto quat = imu_.getQuaternionNoexcept();
        auto gyro = imu_.getGyroscopeNoexcept();
        auto accel = imu_.getLinearAccelerationNoexcept();  // Acceleration excluding gravity

        if (!quat || !gyro || !accel || !mag || !temp) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Communication dropout. Diagnostics: RxErr=%u, TxErr=%u, Reconnects=%u",
                                 imu_.getDiagnostics().read_failures, imu_.getDiagnostics().write_failures,
                                 imu_.getDiagnostics().reconnect_attempts);
            return;
        }

        // Allocate standard message dynamically as a unique_ptr to enable zero-copy intra-process transport.
        auto message = std::make_unique<sensor_msgs::msg::Imu>();

        message->header.stamp = stamp;
        message->header.frame_id = frame_id_;

        // Fill dynamic orientation
        message->orientation.w = quat->w;
        message->orientation.x = quat->x;
        message->orientation.y = quat->y;
        message->orientation.z = quat->z;

        // Fill dynamic angular velocity (rad/s)
        message->angular_velocity.x = gyro->x;
        message->angular_velocity.y = gyro->y;
        message->angular_velocity.z = gyro->z;

        // Fill dynamic linear acceleration (m/s^2)
        message->linear_acceleration.x = accel->x;
        message->linear_acceleration.y = accel->y;
        message->linear_acceleration.z = accel->z;

        // Set covariances from parameters using common helper
        bno055_ros2::fill_imu_covariances(this, *message);

        // Publish using std::move to enable zero-copy pointer pass
        publisher_->publish(std::move(message));

        // Magnetic Field Zero-Copy
        auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();
        mag_msg->header.stamp = stamp;
        mag_msg->header.frame_id = frame_id_;
        mag_msg->magnetic_field.x = mag->x * 1e-6;  // Convert uT to Tesla
        mag_msg->magnetic_field.y = mag->y * 1e-6;
        mag_msg->magnetic_field.z = mag->z * 1e-6;
        bno055_ros2::fill_mag_covariance(this, *mag_msg);
        mag_publisher_->publish(std::move(mag_msg));

        // Temperature
        auto temp_msg = std::make_unique<sensor_msgs::msg::Temperature>();
        temp_msg->header.stamp = stamp;
        temp_msg->header.frame_id = frame_id_;
        temp_msg->temperature = static_cast<double>(*temp);
        temp_msg->variance = this->get_parameter("temperature_variance").as_double();
        temp_publisher_->publish(std::move(temp_msg));
    }

    void publish_diagnostics() {
        auto diag_arr = bno055_ros2::build_diagnostics(this, imu_, "IMU Perf Sensor Monitor");
        diag_publisher_->publish(std::move(diag_arr));
    }

    bno055lib::BNO055 imu_;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_calib_service_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace bno055_ros2

#ifndef ROS2_NODE_TESTING
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // Enable intra-process communication by default for high performance
    options.use_intra_process_comms(true);

    try {
        rclcpp::spin(std::make_shared<bno055_ros2::BNO055PerfPublisherNode>(options));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "High-Performance Node terminated: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
#endif
