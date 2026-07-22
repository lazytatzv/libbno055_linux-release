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
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>
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

    ~BNO055LifecyclePublisherNode() override {
        imu_.stopRawAsyncReading();
        imu_.stopInterruptDrivenReading();
    }

    // Configure Transition: Initialize hardware, setup communications
    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override {
        (void)state;
        std::string device = this->get_parameter("device").as_string();
        int address = this->get_parameter("address").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double rate_hz = this->get_parameter("publish_rate").as_double();
        std::string calib_file = this->get_parameter("calibration_file").as_string();

        std::string connection_type = this->get_parameter("connection_type").as_string();

        if (connection_type == "uart") {
            bno055lib::BNO055::UARTConfig uart_config;
            uart_config.port = this->get_parameter("uart_port").as_string();
            uart_config.baudrate = this->get_parameter("uart_baudrate").as_int();
            uart_config.timeout = this->get_parameter("uart_timeout").as_double();
            uart_config.low_latency = this->get_parameter("uart_low_latency").as_bool();
            RCLCPP_INFO(this->get_logger(), "Initializing BNO055 on UART %s (%d bps, low_latency: %s)",
                        uart_config.port.c_str(), uart_config.baudrate, uart_config.low_latency ? "true" : "false");
            imu_ = bno055lib::BNO055(uart_config);
        } else {
            RCLCPP_INFO(this->get_logger(), "Configuring BNO055 on I2C %s (address: 0x%02X)", device.c_str(), address);
            imu_ = bno055lib::BNO055(address, device);
        }

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

        // Setup automatic calibration saving or manual file load
        bool enable_auto_calib = this->get_parameter("enable_auto_calibration").as_bool();
        if (enable_auto_calib && !calib_file.empty()) {
            imu_.enableAutoCalibration(calib_file);
            RCLCPP_INFO(this->get_logger(), "Automatic calibration loading and saving enabled for: %s",
                        calib_file.c_str());
        } else if (!calib_file.empty()) {
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
        raw_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/raw", qos);
        gravity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu/gravity", qos);
        calib_pub_ = this->create_publisher<std_msgs::msg::String>("~/calib_status", 10);
        status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/status", 10);
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

                bno055lib::CalibrationStatus calib;
                try {
                    calib = imu_.getCalibrationStatus();
                } catch (const std::exception& e) {
                    response->success = false;
                    response->message = std::string("Hardware error checking status: ") + e.what();
                    return;
                }
                if (!calib.isFullyCalibrated()) {
                    response->success = false;
                    response->message = "Refused: Sensor not fully calibrated (S:" + std::to_string(calib.sys) +
                                        " G:" + std::to_string(calib.gyro) + " A:" + std::to_string(calib.accel) +
                                        " M:" + std::to_string(calib.mag) + ").";
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

        calib_request_service_ = this->create_service<std_srvs::srv::Trigger>(
            "~/calibration_request", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                try {
                    auto status = imu_.getCalibrationStatus();
                    char buf[128];
                    snprintf(buf, sizeof(buf), "{\"sys\": %d, \"gyro\": %d, \"accel\": %d, \"mag\": %d}", status.sys,
                             status.gyro, status.accel, status.mag);
                    response->success = true;
                    response->message = buf;
                } catch (const std::exception& e) {
                    response->success = false;
                    response->message = std::string("Hardware error: ") + e.what();
                }
            });

        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                if (imu_.reset()) {
                    response->success = true;
                    response->message = "IMU hardware reset successful";
                } else {
                    response->success = false;
                    response->message = "IMU hardware reset failed";
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
        mag_publisher_->on_activate();
        temp_publisher_->on_activate();
        raw_publisher_->on_activate();
        gravity_publisher_->on_activate();
        calib_pub_->on_activate();
        status_pub_->on_activate();
        diag_publisher_->on_activate();

        std::string read_mode = this->get_parameter("read_mode").as_string();
        int gpio_pin = this->get_parameter("interrupt_gpio_pin").as_int();
        double rate_hz = this->get_parameter("publish_rate").as_double();

        if (read_mode == "raw_async") {
            RCLCPP_INFO(this->get_logger(), "Activating raw_async mode at %.1f Hz", rate_hz);
            imu_.startRawAsyncReading(
                rate_hz, std::bind(&BNO055LifecyclePublisherNode::raw_data_callback, this, std::placeholders::_1));
        } else if (read_mode == "interrupt") {
            RCLCPP_INFO(this->get_logger(), "Activating hardware interrupt mode on GPIO %d", gpio_pin);
            imu_.startInterruptDrivenReading(
                gpio_pin, std::bind(&BNO055LifecyclePublisherNode::raw_data_callback, this, std::placeholders::_1));
        } else {
            RCLCPP_INFO(this->get_logger(), "Activating standard timer polling mode");
            timer_->reset();
        }

        diag_timer_->reset();

        RCLCPP_INFO(this->get_logger(), "Activation successful. Publishing started.");
        return CallbackReturn::SUCCESS;
    }

    // Deactivate Transition: Transition to inactive state, stop publishing and suspend hardware
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Deactivating BNO055...");

        // Stop background reading threads if running
        imu_.stopRawAsyncReading();
        imu_.stopInterruptDrivenReading();

        // Stop timers
        timer_->cancel();
        diag_timer_->cancel();

        // Deactivate publishers
        publisher_->on_deactivate();
        mag_publisher_->on_deactivate();
        temp_publisher_->on_deactivate();
        raw_publisher_->on_deactivate();
        gravity_publisher_->on_deactivate();
        calib_pub_->on_deactivate();
        status_pub_->on_deactivate();
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

        imu_.stopRawAsyncReading();
        imu_.stopInterruptDrivenReading();

        // Release structures
        timer_.reset();
        diag_timer_.reset();
        publisher_.reset();
        mag_publisher_.reset();
        temp_publisher_.reset();
        raw_publisher_.reset();
        gravity_publisher_.reset();
        calib_pub_.reset();
        status_pub_.reset();
        diag_publisher_.reset();
        save_calib_service_.reset();
        calib_request_service_.reset();
        reset_srv_.reset();

        RCLCPP_INFO(this->get_logger(), "Cleanup successful.");
        return CallbackReturn::SUCCESS;
    }

    // Shutdown Transition: Handle final termination
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Shutting down BNO055 Node...");

        imu_.stopRawAsyncReading();
        imu_.stopInterruptDrivenReading();

        // Ensure device is suspended
        imu_.enterSuspendMode();

        timer_.reset();
        diag_timer_.reset();
        publisher_.reset();
        mag_publisher_.reset();
        temp_publisher_.reset();
        raw_publisher_.reset();
        gravity_publisher_.reset();
        calib_pub_.reset();
        status_pub_.reset();
        diag_publisher_.reset();
        save_calib_service_.reset();
        calib_request_service_.reset();
        reset_srv_.reset();

        return CallbackReturn::SUCCESS;
    }

private:
    void timer_callback() {
        // Record timestamp immediately before I2C communication starts to minimize jitter
        auto stamp = this->now();

        // Noexcept reads
        auto quat = imu_.getQuaternionNoexcept();
        auto gyro = imu_.getGyroscopeNoexcept();
        auto accel = imu_.getLinearAccelerationNoexcept();
        auto mag = imu_.getMagnetometerNoexcept();
        auto temp = imu_.getTemperatureNoexcept();
        auto raw_accel = imu_.getAccelerometerNoexcept();
        auto grav = imu_.getGravityNoexcept();

        if (!quat || !gyro || !accel || !mag || !temp || !raw_accel || !grav) {
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

        // Raw Data (Unfiltered)
        auto raw_msg = std::make_unique<sensor_msgs::msg::Imu>();
        raw_msg->header.stamp = stamp;
        raw_msg->header.frame_id = frame_id_;
        raw_msg->linear_acceleration.x = raw_accel->x;
        raw_msg->linear_acceleration.y = raw_accel->y;
        raw_msg->linear_acceleration.z = raw_accel->z;
        raw_msg->angular_velocity.x = gyro->x;
        raw_msg->angular_velocity.y = gyro->y;
        raw_msg->angular_velocity.z = gyro->z;
        raw_msg->orientation.w = 1.0;  // Raw typically omits fusion orientation
        raw_publisher_->publish(std::move(raw_msg));

        // Gravity Vector
        auto grav_msg = std::make_unique<geometry_msgs::msg::Vector3>();
        grav_msg->x = grav->x;
        grav_msg->y = grav->y;
        grav_msg->z = grav->z;
        gravity_publisher_->publish(std::move(grav_msg));
    }

    void raw_data_callback(const bno055lib::BNO055::RawSensorData& data) {
        // Guard callback execution to only run when node is in ACTIVE lifecycle state
        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            return;
        }
        auto stamp = this->now();

        // 1. Publish to imu/raw (Raw sensor readings)
        auto raw_msg = std::make_unique<sensor_msgs::msg::Imu>();
        raw_msg->header.stamp = stamp;
        raw_msg->header.frame_id = frame_id_;

        raw_msg->linear_acceleration.x = data.accel.x;
        raw_msg->linear_acceleration.y = data.accel.y;
        raw_msg->linear_acceleration.z = data.accel.z;

        raw_msg->angular_velocity.x = data.gyro.x;
        raw_msg->angular_velocity.y = data.gyro.y;
        raw_msg->angular_velocity.z = data.gyro.z;

        raw_msg->orientation.w = 1.0;  // Fill identity orientation for raw topic
        bno055_ros2::fill_imu_covariances(this, *raw_msg);
        raw_publisher_->publish(std::move(raw_msg));

        // 2. Publish to imu/mag
        auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();
        mag_msg->header.stamp = stamp;
        mag_msg->header.frame_id = frame_id_;
        mag_msg->magnetic_field.x = data.mag.x * 1e-6;
        mag_msg->magnetic_field.y = data.mag.y * 1e-6;
        mag_msg->magnetic_field.z = data.mag.z * 1e-6;
        bno055_ros2::fill_mag_covariance(this, *mag_msg);
        mag_publisher_->publish(std::move(mag_msg));
    }

    void publish_diagnostics() {
        auto diag_arr = bno055_ros2::build_diagnostics(this, imu_, "IMU Lifecycle Sensor Monitor");
        diag_publisher_->publish(std::move(diag_arr));

        bno055lib::CalibrationStatus status;
        try {
            status = imu_.getCalibrationStatus();
            char buf[128];
            snprintf(buf, sizeof(buf), "{\"sys\": %d, \"gyro\": %d, \"accel\": %d, \"mag\": %d}", status.sys,
                     status.gyro, status.accel, status.mag);
            auto calib_msg = std::make_unique<std_msgs::msg::String>();
            calib_msg->data = buf;
            calib_pub_->publish(std::move(calib_msg));
        } catch (...) {
            // If it fails, default to 0 to gracefully degrade the status message
            status.sys = 0;
            status.gyro = 0;
            status.accel = 0;
            status.mag = 0;
        }

        // Publish DiagnosticStatus
        auto status_msg = diagnostic_msgs::msg::DiagnosticStatus();
        status_msg.name = this->get_name();
        status_msg.hardware_id = "BNO055";
        status_msg.level = (status.sys == 3) ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                             : diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status_msg.message = "Sys: " + std::to_string(status.sys) + " Gyro: " + std::to_string(status.gyro) +
                             " Accel: " + std::to_string(status.accel) + " Mag: " + std::to_string(status.mag);
        status_pub_->publish(status_msg);
    }

    bno055lib::BNO055 imu_;
    std::string frame_id_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr raw_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3>::SharedPtr gravity_publisher_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr calib_pub_;
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_calib_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_request_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace bno055_ros2

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(bno055_ros2::BNO055LifecyclePublisherNode)

#ifndef BNO055_ROS2_BUILDING_COMPONENT
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
#endif
