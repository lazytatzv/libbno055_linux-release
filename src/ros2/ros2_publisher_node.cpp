/**
 * @file ros2_publisher_node.cpp
 * @brief Standard standalone ROS 2 publisher node for the BNO055 IMU sensor.
 *
 * This node initializes the BNO055 sensor in IMUPlus fusion mode (6-axis),
 * declares standard ROS 2 parameters for runtime configuration, redirects
 * internal library logs to RCLCPP, and publishes IMU telemetry data
 * (sensor_msgs/msg/Imu) at a configurable frequency.
 */

#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "bno055_ros2_common.hpp"
#include "libbno055-linux/bno055.hpp"

using namespace std::chrono_literals;

class BNO055PublisherNode : public rclcpp::Node {
public:
    explicit BNO055PublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("bno055_publisher_node", options) {
        // Declare and fetch parameters using common helpers
        bno055_ros2::declare_common_parameters(this);

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
            RCLCPP_INFO(this->get_logger(), "Initializing BNO055 on UART %s (%d bps)", uart_config.port.c_str(),
                        uart_config.baudrate);
            imu_.emplace(uart_config);
        } else {
            RCLCPP_INFO(this->get_logger(), "Initializing BNO055 on I2C %s (address: 0x%02X)", device.c_str(), address);
            imu_.emplace(address, device);
        }

        // Redirect library internal logs into ROS 2 RCLCPP logger
        bno055_ros2::setup_logger_redirection(this, *imu_);

        auto mode = bno055_ros2::parse_op_mode(this->get_parameter("operation_mode").as_string());
        if (!imu_->begin(mode)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize BNO055 sensor!");
            throw std::runtime_error("Sensor initialization failed");
        }

        // Apply advanced hardware configurations
        bno055_ros2::apply_advanced_features(this, *imu_);

        // Setup automatic calibration saving or manual file load
        bool enable_auto_calib = this->get_parameter("enable_auto_calibration").as_bool();
        if (enable_auto_calib && !calib_file.empty()) {
            imu_->enableAutoCalibration(calib_file);
            RCLCPP_INFO(this->get_logger(), "Automatic calibration loading and saving enabled for: %s",
                        calib_file.c_str());
        } else if (!calib_file.empty()) {
            if (imu_->loadCalibrationFile(calib_file)) {
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
        raw_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/raw", qos);
        gravity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu/gravity", qos);
        calib_pub_ = this->create_publisher<std_msgs::msg::String>("~/calib_status", qos);
        status_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/status", qos);

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
                if (imu_->saveCalibrationFile(cf)) {
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
                auto status = imu_->getCalibrationStatus();
                char buf[128];
                snprintf(buf, sizeof(buf), "{\"sys\": %d, \"gyro\": %d, \"accel\": %d, \"mag\": %d}", status.sys,
                         status.gyro, status.accel, status.mag);
                response->success = true;
                response->message = buf;
            });

        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                if (imu_ && imu_->reset()) {
                    response->success = true;
                    response->message = "IMU hardware reset successful";
                } else {
                    response->success = false;
                    response->message = "IMU hardware reset failed";
                }
            });

        diag_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        std::string read_mode = this->get_parameter("read_mode").as_string();
        int gpio_pin = this->get_parameter("interrupt_gpio_pin").as_int();

        if (read_mode == "raw_async") {
            RCLCPP_INFO(this->get_logger(), "Starting in High-Performance RAW ASYNC Mode at %.1f Hz", rate_hz);
            imu_->startRawAsyncReading(rate_hz,
                                       std::bind(&BNO055PublisherNode::raw_data_callback, this, std::placeholders::_1));
        } else if (read_mode == "interrupt") {
            RCLCPP_INFO(this->get_logger(), "Starting in High-Performance HARDWARE INTERRUPT (IRQ) Mode on GPIO Pin %d",
                        gpio_pin);
            imu_->startInterruptDrivenReading(
                gpio_pin, std::bind(&BNO055PublisherNode::raw_data_callback, this, std::placeholders::_1));
        } else {
            RCLCPP_INFO(this->get_logger(), "Starting in STANDARD Polling Mode at %.1f Hz", rate_hz);
            auto interval = std::chrono::duration<double>(1.0 / rate_hz);
            timer_ = this->create_wall_timer(interval, std::bind(&BNO055PublisherNode::timer_callback, this));
        }

        diag_timer_ = this->create_wall_timer(1s, std::bind(&BNO055PublisherNode::publish_diagnostics, this));
    }

private:
    void timer_callback() {
        // Record timestamp immediately before I2C communication starts to minimize jitter
        auto stamp = this->now();

        // Exception-free (noexcept) read path ensures no CPU overhead on communication drops
        auto quat = imu_->getQuaternionNoexcept();
        auto gyro = imu_->getGyroscopeNoexcept();
        auto accel = imu_->getLinearAccelerationNoexcept();
        auto mag = imu_->getMagnetometerNoexcept();
        auto temp = imu_->getTemperatureNoexcept();  // Acceleration excluding gravity
        auto raw_accel = imu_->getAccelerometerNoexcept();
        auto grav = imu_->getGravityNoexcept();

        if (!quat || !gyro || !accel || !mag || !temp || !raw_accel || !grav) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Communication dropout detected. Diagnostics: RxErr=%u, TxErr=%u, Reconnects=%u",
                                 imu_->getDiagnostics().read_failures, imu_->getDiagnostics().write_failures,
                                 imu_->getDiagnostics().reconnect_attempts);
            return;
        }

        // Use std::unique_ptr for Zero-Copy intra-process communication optimization
        auto message = std::make_unique<sensor_msgs::msg::Imu>();
        message->header.stamp = stamp;
        message->header.frame_id = frame_id_;

        // Fill Orientation
        message->orientation.w = quat->w;
        message->orientation.x = quat->x;
        message->orientation.y = quat->y;
        message->orientation.z = quat->z;

        // Fill Angular Velocity (rad/s)
        message->angular_velocity.x = gyro->x;
        message->angular_velocity.y = gyro->y;
        message->angular_velocity.z = gyro->z;

        // Fill Linear Acceleration (m/s^2)
        message->linear_acceleration.x = accel->x;
        message->linear_acceleration.y = accel->y;
        message->linear_acceleration.z = accel->z;

        // Set covariances from parameters using common helper
        bno055_ros2::fill_imu_covariances(this, *message);

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
        auto diag_arr = bno055_ros2::build_diagnostics(this, *imu_, "IMU Sensor Monitor");
        diag_publisher_->publish(*diag_arr);

        auto status = imu_->getCalibrationStatus();
        char buf[128];
        snprintf(buf, sizeof(buf), "{\"sys\": %d, \"gyro\": %d, \"accel\": %d, \"mag\": %d}", status.sys, status.gyro,
                 status.accel, status.mag);
        auto calib_status_msg = std::make_unique<std_msgs::msg::String>();
        calib_status_msg->data = buf;
        calib_pub_->publish(std::move(calib_status_msg));

        // Publish DiagnosticStatus for compatibility
        auto status_msg = diagnostic_msgs::msg::DiagnosticStatus();
        status_msg.name = this->get_name();
        status_msg.hardware_id = "BNO055";
        status_msg.level = (status.sys == 3) ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                             : diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status_msg.message = "Sys: " + std::to_string(status.sys) + " Gyro: " + std::to_string(status.gyro) +
                             " Accel: " + std::to_string(status.accel) + " Mag: " + std::to_string(status.mag);
        status_pub_->publish(status_msg);
    }

public:
    ~BNO055PublisherNode() override {
        if (imu_) {
            imu_->stopRawAsyncReading();
            imu_->stopInterruptDrivenReading();
        }
    }

private:
    std::optional<bno055lib::BNO055> imu_;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gravity_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calib_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_calib_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calib_request_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
};

#ifndef ROS2_NODE_TESTING
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // Enable intra-process communication by default for high performance zero-copy
    options.use_intra_process_comms(true);

    try {
        rclcpp::spin(std::make_shared<BNO055PublisherNode>(options));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node terminated due to exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
#endif
