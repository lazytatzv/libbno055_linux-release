/**
 * @file ros2_lifecycle_publisher_node.cpp
 * @brief Managed Lifecycle (State Machine) ROS 2 node for the BNO055 sensor.
 *
 * This node implements rclcpp_lifecycle::LifecycleNode to fit into managed
 * robot startup and shutdown sequences. It maps the physical states of the
 * BNO055 hardware to ROS 2 lifecycle states:
 * - on_configure: Boot the sensor and put it into low-power suspend mode.
 * - on_activate: Wake up the sensor to normal mode and start the timer/publisher.
 * - on_deactivate: Stop publishing and put the hardware back to suspend mode.
 * - on_cleanup: Release I2C file descriptors and reset resources.
 */

#include <algorithm>
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <utility>

#include "bno055_ros2_common.hpp"
#include "libbno055-linux/bno055.hpp"

using namespace std::chrono_literals;

namespace bno055_ros2 {

class BNO055LifecyclePublisherNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit BNO055LifecyclePublisherNode(const rclcpp::NodeOptions& options)
        : LifecycleNode("bno055_lifecycle_publisher_node", options), imu_(0x28, "/dev/i2c-1") {
        // Declare standard parameters using common helper
        bno055_ros2::declare_common_parameters(this);
    }

    // Configure Transition: Initialize hardware, setup communications
    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override {
        (void)state;
        std::string device = this->get_parameter("device").as_string();
        int address = this->get_parameter("address").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double rate_hz = this->get_parameter("publish_rate").as_double();
        std::string calib_file = this->get_parameter("calibration_file").as_string();

        RCLCPP_INFO(this->get_logger(), "Configuring BNO055: device=%s, address=0x%02X", device.c_str(), address);

        imu_ = bno055lib::BNO055(address, device);

        // Redirect internal logs into ROS 2 RCLCPP
        bno055_ros2::setup_logger_redirection(this, imu_);

        // Initialize BNO055
        auto mode = bno055_ros2::parse_op_mode(this->get_parameter("operation_mode").as_string());
        if (!imu_.begin(mode)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO055 during configuration!");
            return CallbackReturn::FAILURE;
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

        // Put device to low power suspend mode until activated to save energy
        imu_.enterSuspendMode();

        // Setup dynamic QoS
        auto qos = rclcpp::SensorDataQoS();
        std::string reliability = this->get_parameter("qos_reliability").as_string();
        if (reliability == "reliable") {
            qos.reliable();
        } else if (reliability == "best_effort") {
            qos.best_effort();
        }
        qos.keep_last(this->get_parameter("qos_history_depth").as_int());

        // Create lifecycle publishers
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

        // Setup timers
        auto interval = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(interval, std::bind(&BNO055LifecyclePublisherNode::timer_callback, this));
        diag_timer_ = this->create_wall_timer(1s, std::bind(&BNO055LifecyclePublisherNode::publish_diagnostics, this));

        // Explicitly cancel the timers so they don't run until active
        timer_->cancel();
        diag_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Configuration successful. Device suspended. Ready to activate.");
        return CallbackReturn::SUCCESS;
    }

    // Activate Transition: Transition to active state, resume hardware and resume publishing
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Activating BNO055...");

        // Wake up BNO055 sensor
        imu_.enterNormalMode();

        // Activate lifecycle publishers
        publisher_->on_activate();
        diag_publisher_->on_activate();

        // Restart timers
        timer_->reset();
        diag_timer_->reset();

        RCLCPP_INFO(this->get_logger(), "Activation successful. Publishing started.");
        return CallbackReturn::SUCCESS;
    }

    // Deactivate Transition: Transition to inactive state, stop publishing and suspend hardware
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Deactivating BNO055...");

        // Stop timers
        timer_->cancel();
        diag_timer_->cancel();

        // Deactivate publishers
        publisher_->on_deactivate();
        diag_publisher_->on_deactivate();

        // Suspend sensor to save power
        imu_.enterSuspendMode();

        RCLCPP_INFO(this->get_logger(), "Deactivation successful. Sensor suspended.");
        return CallbackReturn::SUCCESS;
    }

    // Cleanup Transition: Clean up resources
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Cleaning up BNO055 resources...");

        // Release structures
        timer_.reset();
        diag_timer_.reset();
        publisher_.reset();
        diag_publisher_.reset();

        RCLCPP_INFO(this->get_logger(), "Cleanup successful.");
        return CallbackReturn::SUCCESS;
    }

    // Shutdown Transition: Handle final termination
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Shutting down BNO055 Node...");

        // Ensure device is suspended
        imu_.enterSuspendMode();

        timer_.reset();
        diag_timer_.reset();
        publisher_.reset();
        diag_publisher_.reset();

        return CallbackReturn::SUCCESS;
    }

private:
    void timer_callback() {
        // Record timestamp immediately before I2C communication starts to minimize jitter
        auto stamp = this->now();

        // High-performance noexcept reads
        auto quat = imu_.getQuaternionNoexcept();
        auto gyro = imu_.getGyroscopeNoexcept();
        auto accel = imu_.getLinearAccelerationNoexcept();

        if (!quat || !gyro || !accel || !mag || !temp) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Communication dropout (Lifecycle). Diagnostics: RxErr=%u, TxErr=%u, Reconnects=%u",
                                 imu_.getDiagnostics().read_failures, imu_.getDiagnostics().write_failures,
                                 imu_.getDiagnostics().reconnect_attempts);
            return;
        }

        // Allocate std::unique_ptr for zero-copy intra-process transport
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

        // Publish using std::move to enable zero-copy intra-process transport
        publisher_->publish(std::move(message));

        // Magnetic Field
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
        auto diag_arr = bno055_ros2::build_diagnostics(this, imu_, "IMU Lifecycle Sensor Monitor");
        diag_publisher_->publish(std::move(diag_arr));
    }

    bno055lib::BNO055 imu_;
    std::string frame_id_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher_;
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

    auto node = std::make_shared<bno055_ros2::BNO055LifecyclePublisherNode>(options);

    try {
        rclcpp::spin(node->get_node_base_interface());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Lifecycle Node terminated: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
#endif
