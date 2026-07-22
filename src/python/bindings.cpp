#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "libbno055-linux/bno055.hpp"

namespace py = pybind11;
using namespace bno055lib;

PYBIND11_MODULE(libbno055, m) {
    m.doc() = "Python bindings for libbno055-linux C++ library";

    // OpMode enum
    py::enum_<OpMode>(m, "OpMode")
        .value("Config", OpMode::Config)
        .value("AccOnly", OpMode::AccOnly)
        .value("MagOnly", OpMode::MagOnly)
        .value("GyroOnly", OpMode::GyroOnly)
        .value("AccMag", OpMode::AccMag)
        .value("AccGyro", OpMode::AccGyro)
        .value("MagGyro", OpMode::MagGyro)
        .value("AMG", OpMode::AMG)
        .value("IMUPlus", OpMode::IMUPlus)
        .value("Compass", OpMode::Compass)
        .value("M4G", OpMode::M4G)
        .value("NDOF_FMC_Off", OpMode::NDOF_FMC_Off)
        .value("NDOF", OpMode::NDOF)
        .export_values();

    // Vector3 struct
    py::class_<Vector3>(m, "Vector3")
        .def(py::init<float, float, float>(), py::arg("x") = 0.0f, py::arg("y") = 0.0f, py::arg("z") = 0.0f)
        .def_readwrite("x", &Vector3::x)
        .def_readwrite("y", &Vector3::y)
        .def_readwrite("z", &Vector3::z)
        .def("__repr__", [](const Vector3& v) {
            return "<Vector3 x=" + std::to_string(v.x) + " y=" + std::to_string(v.y) + " z=" + std::to_string(v.z) +
                   ">";
        });

    // Quaternion struct
    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<float, float, float, float>(), py::arg("w") = 1.0f, py::arg("x") = 0.0f, py::arg("y") = 0.0f,
             py::arg("z") = 0.0f)
        .def_readwrite("w", &Quaternion::w)
        .def_readwrite("x", &Quaternion::x)
        .def_readwrite("y", &Quaternion::y)
        .def_readwrite("z", &Quaternion::z)
        .def("__repr__", [](const Quaternion& q) {
            return "<Quaternion w=" + std::to_string(q.w) + " x=" + std::to_string(q.x) + " y=" + std::to_string(q.y) +
                   " z=" + std::to_string(q.z) + ">";
        });

    // CalibrationStatus struct
    py::class_<CalibrationStatus>(m, "CalibrationStatus")
        .def_readwrite("sys", &CalibrationStatus::sys)
        .def_readwrite("gyro", &CalibrationStatus::gyro)
        .def_readwrite("accel", &CalibrationStatus::accel)
        .def_readwrite("mag", &CalibrationStatus::mag)
        .def("is_fully_calibrated", &CalibrationStatus::isFullyCalibrated)
        .def("__repr__", [](const CalibrationStatus& c) {
            return "<CalibrationStatus sys=" + std::to_string(c.sys) + " gyro=" + std::to_string(c.gyro) +
                   " accel=" + std::to_string(c.accel) + " mag=" + std::to_string(c.mag) + ">";
        });

    // Diagnostics struct
    py::class_<Diagnostics>(m, "Diagnostics")
        .def_readwrite("write_failures", &Diagnostics::write_failures)
        .def_readwrite("read_failures", &Diagnostics::read_failures)
        .def_readwrite("reconnect_attempts", &Diagnostics::reconnect_attempts)
        .def("__repr__", [](const Diagnostics& d) {
            return "<Diagnostics write_failures=" + std::to_string(d.write_failures) +
                   " read_failures=" + std::to_string(d.read_failures) +
                   " reconnect_attempts=" + std::to_string(d.reconnect_attempts) + ">";
        });

    // RawSensorData struct
    py::class_<BNO055::RawSensorData>(m, "RawSensorData")
        .def_readwrite("accel", &BNO055::RawSensorData::accel)
        .def_readwrite("mag", &BNO055::RawSensorData::mag)
        .def_readwrite("gyro", &BNO055::RawSensorData::gyro);

    // BNO055 class
    py::class_<BNO055>(m, "BNO055")
        .def(py::init<uint8_t, std::string_view>(), py::arg("address") = 0x28, py::arg("device") = "/dev/i2c-1")
        .def("begin", &BNO055::begin, py::arg("mode") = OpMode::NDOF)
        .def("reset", &BNO055::reset)
        .def("set_mode", &BNO055::setMode, py::arg("mode"))
        .def("get_mode", &BNO055::getMode)
        .def("set_ext_crystal_use", &BNO055::setExtCrystalUse, py::arg("use_xtal"))
        // Data getters (noexcept std::optional bindings)
        .def("get_accelerometer", &BNO055::getAccelerometerNoexcept)
        .def("get_magnetometer", &BNO055::getMagnetometerNoexcept)
        .def("get_gyroscope", &BNO055::getGyroscopeNoexcept)
        .def("get_euler_angles", &BNO055::getEulerAnglesNoexcept)
        .def("get_linear_acceleration", &BNO055::getLinearAccelerationNoexcept)
        .def("get_gravity", &BNO055::getGravityNoexcept)
        .def("get_quaternion", &BNO055::getQuaternionNoexcept)
        .def("get_temperature", &BNO055::getTemperatureNoexcept)
        .def("get_raw_sensor_data", &BNO055::getRawSensorDataNoexcept)
        // Calibration & Diagnostics
        .def("get_calibration_status",
             [](BNO055& self) {
                 try {
                     return std::optional<CalibrationStatus>(self.getCalibrationStatus());
                 } catch (...) {
                     return std::optional<CalibrationStatus>(std::nullopt);
                 }
             })
        .def("get_diagnostics", &BNO055::getDiagnostics)
        .def("save_calibration_file", &BNO055::saveCalibrationFile, py::arg("filepath"))
        .def("load_calibration_file", &BNO055::loadCalibrationFile, py::arg("filepath"))
        .def("enable_auto_calibration", &BNO055::enableAutoCalibration, py::arg("filepath"))
        .def("disable_auto_calibration", &BNO055::disableAutoCalibration)
        .def("get_sensor_offsets",
             [](BNO055& self) {
                 std::array<uint8_t, 22> calib_data;
                 if (self.getSensorOffsets(calib_data)) {
                     return calib_data;
                 }
                 throw std::runtime_error("Failed to get sensor offsets");
             })
        .def(
            "set_sensor_offsets",
            [](BNO055& self, const std::array<uint8_t, 22>& data) { self.setSensorOffsets(data); }, py::arg("data"))
        .def("enter_suspend_mode",
             [](BNO055& self) {
                 try {
                     self.enterSuspendMode();
                 } catch (...) {
                 }
             })
        .def("enter_normal_mode", [](BNO055& self) {
            try {
                self.enterNormalMode();
            } catch (...) {
            }
        });

    // Utility function
    m.def("to_euler_degrees", &toEulerDegrees, py::arg("q"), "Convert Quaternion to Euler angles in degrees");
}
