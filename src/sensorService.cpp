#include <memory>
#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <chrono>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/trigger.hpp"

#include "../include/registers.hpp"
#include "../include/i2c.hpp"

using namespace std::chrono_literals;

// Simple parameter wrapper to mimic Python's .value access
template <typename T>
struct Param {
  T value{};
};

// NodeParameters equivalent for C++
struct NodeParameters {
  Param<std::string> ros_topic_prefix;
  Param<std::string> placement_axis_remap; // e.g., "P0".."P7"
  Param<bool> set_offsets;
  Param<uint8_t> operation_mode; // registers::OPERATION_MODE_*
  Param<std::string> frame_id;

  Param<std::array<double, 3>> variance_orientation;
  Param<std::array<double, 3>> variance_acc;
  Param<std::array<double, 3>> variance_angular_vel;
  Param<std::array<double, 3>> variance_mag;

  Param<double> acc_factor;
  Param<double> gyr_factor;
  Param<double> mag_factor;
  Param<double> grav_factor;

  Param<std::array<int16_t, 3>> offset_acc;
  Param<std::array<int16_t, 3>> offset_mag;
  Param<std::array<int16_t, 3>> offset_gyr;
  Param<uint16_t> radius_mag;
  Param<uint16_t> radius_acc;
};

class SensorService : public std::enable_shared_from_this<SensorService> {
public:
  SensorService(std::shared_ptr<rclcpp::Node> node,
                std::shared_ptr<Connector> connector,
                const NodeParameters &param)
      : node_(std::move(node)), con_(std::move(connector)), param_(param) {
    const auto &prefix = param_.ros_topic_prefix.value;
    rclcpp::QoS qos(10);

    pub_imu_raw_ = node_->create_publisher<sensor_msgs::msg::Imu>(prefix + "imu_raw", qos);
    pub_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>(prefix + "imu", qos);
    pub_mag_ = node_->create_publisher<sensor_msgs::msg::MagneticField>(prefix + "mag", qos);
    pub_grav_ = node_->create_publisher<geometry_msgs::msg::Vector3>(prefix + "grav", qos);
    pub_temp_ = node_->create_publisher<sensor_msgs::msg::Temperature>(prefix + "temp", qos);
    pub_calib_status_ = node_->create_publisher<std_msgs::msg::String>(prefix + "calib_status", qos);

    srv_ = node_->create_service<example_interfaces::srv::Trigger>(
        prefix + "calibration_request",
        std::bind(&SensorService::calibration_request_callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void configure() {
    RCLCPP_INFO(node_->get_logger(), "Configuring device...");
    try {
      auto data = con_->receive(BNO055_CHIP_ID_ADDR, 1);
      if (data.empty() || data[0] != BNO055_ID) {
        throw std::runtime_error("Device ID incorrect");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "Communication error: %s", e.what());
      RCLCPP_ERROR(node_->get_logger(), "Shutting down ROS node...");
      std::exit(1);
    }

    // IMU connected => apply configuration
    if (!con_->transmit(BNO055_OPR_MODE_ADDR, 1, std::vector<uint8_t>{OPERATION_MODE_CONFIG})) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set IMU into config mode.");
    }

    if (!con_->transmit(BNO055_PWR_MODE_ADDR, 1, std::vector<uint8_t>{POWER_MODE_NORMAL})) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set IMU normal power mode.");
    }

    if (!con_->transmit(BNO055_PAGE_ID_ADDR, 1, std::vector<uint8_t>{0x00})) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set IMU register page 0.");
    }

    if (!con_->transmit(BNO055_SYS_TRIGGER_ADDR, 1, std::vector<uint8_t>{0x00})) {
      RCLCPP_WARN(node_->get_logger(), "Unable to start IMU.");
    }

    if (!con_->transmit(BNO055_UNIT_SEL_ADDR, 1, std::vector<uint8_t>{0x83})) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set IMU units.");
    }

    // Axis remap
    const std::unordered_map<std::string, std::array<uint8_t, 2>> mount_positions{
        {"P0", {0x21, 0x04}}, {"P1", {0x24, 0x00}}, {"P2", {0x24, 0x06}}, {"P3", {0x21, 0x02}},
        {"P4", {0x24, 0x03}}, {"P5", {0x21, 0x02}}, {"P6", {0x21, 0x07}}, {"P7", {0x24, 0x05}},
    };

    auto it = mount_positions.find(param_.placement_axis_remap.value);
    const auto pos = (it != mount_positions.end()) ? it->second : mount_positions.begin()->second;

    if (!con_->transmit(BNO055_AXIS_MAP_CONFIG_ADDR, 2, std::vector<uint8_t>{pos[0], pos[1]})) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set sensor placement configuration.");
    }

    // Show current sensor offsets
    RCLCPP_INFO(node_->get_logger(), "Current sensor offsets:");
    print_calib_data();

    if (param_.set_offsets.value) {
      bool ok = set_calib_offsets(param_.offset_acc, param_.offset_mag, param_.offset_gyr,
                                  param_.radius_mag, param_.radius_acc);
      if (ok) {
        RCLCPP_INFO(node_->get_logger(), "Successfully configured sensor offsets to:");
        print_calib_data();
      } else {
        RCLCPP_WARN(node_->get_logger(), "Setting offsets failed");
      }
    }

    // Set Device mode
    uint8_t device_mode = param_.operation_mode.value;
    RCLCPP_INFO(node_->get_logger(), "Setting device_mode to %u", device_mode);

    if (!con_->transmit(BNO055_OPR_MODE_ADDR, 1, std::vector<uint8_t>{device_mode})) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set IMU operation mode into operation mode.");
    }

    RCLCPP_INFO(node_->get_logger(), "Bosch BNO055 IMU configuration complete.");
  }

  void get_sensor_data() {
    using sensor_msgs::msg::Imu;
    using sensor_msgs::msg::MagneticField;
    using sensor_msgs::msg::Temperature;
    using geometry_msgs::msg::Vector3;

    Imu imu_raw_msg;
    Imu imu_msg;
    MagneticField mag_msg;
    Vector3 grav_msg;
    Temperature temp_msg;

    // read from sensor
    auto buf = con_->receive(BNO055_ACCEL_DATA_X_LSB_ADDR, 45);
    if (buf.size() < 45) {
      RCLCPP_WARN(node_->get_logger(), "Received insufficient data length: %zu", buf.size());
      return;
    }

    // Publish raw data
    imu_raw_msg.header.stamp = node_->get_clock()->now().to_msg();
    imu_raw_msg.header.frame_id = param_.frame_id.value;

    // orientation covariance
    imu_raw_msg.orientation_covariance[0] = param_.variance_orientation.value[0];
    imu_raw_msg.orientation_covariance[4] = param_.variance_orientation.value[1];
    imu_raw_msg.orientation_covariance[8] = param_.variance_orientation.value[2];

    imu_raw_msg.linear_acceleration.x = unpackBytesToFloat(buf[0], buf[1]) / param_.acc_factor.value;
    imu_raw_msg.linear_acceleration.y = unpackBytesToFloat(buf[2], buf[3]) / param_.acc_factor.value;
    imu_raw_msg.linear_acceleration.z = unpackBytesToFloat(buf[4], buf[5]) / param_.acc_factor.value;

    imu_raw_msg.linear_acceleration_covariance[0] = param_.variance_acc.value[0];
    imu_raw_msg.linear_acceleration_covariance[4] = param_.variance_acc.value[1];
    imu_raw_msg.linear_acceleration_covariance[8] = param_.variance_acc.value[2];

    imu_raw_msg.angular_velocity.x = unpackBytesToFloat(buf[12], buf[13]) / param_.gyr_factor.value;
    imu_raw_msg.angular_velocity.y = unpackBytesToFloat(buf[14], buf[15]) / param_.gyr_factor.value;
    imu_raw_msg.angular_velocity.z = unpackBytesToFloat(buf[16], buf[17]) / param_.gyr_factor.value;

    imu_raw_msg.angular_velocity_covariance[0] = param_.variance_angular_vel.value[0];
    imu_raw_msg.angular_velocity_covariance[4] = param_.variance_angular_vel.value[1];
    imu_raw_msg.angular_velocity_covariance[8] = param_.variance_angular_vel.value[2];

    pub_imu_raw_->publish(imu_raw_msg);

    // Publish filtered data
    imu_msg.header.stamp = node_->get_clock()->now().to_msg();
    imu_msg.header.frame_id = param_.frame_id.value;

    double qw = unpackBytesToFloat(buf[24], buf[25]);
    double qx = unpackBytesToFloat(buf[26], buf[27]);
    double qy = unpackBytesToFloat(buf[28], buf[29]);
    double qz = unpackBytesToFloat(buf[30], buf[31]);

    double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (norm > 1e-9) {
      imu_msg.orientation.x = qx / norm;
      imu_msg.orientation.y = qy / norm;
      imu_msg.orientation.z = qz / norm;
      imu_msg.orientation.w = qw / norm;
    }

    imu_msg.orientation_covariance = imu_raw_msg.orientation_covariance;

    imu_msg.linear_acceleration.x = unpackBytesToFloat(buf[32], buf[33]) / param_.acc_factor.value;
    imu_msg.linear_acceleration.y = unpackBytesToFloat(buf[34], buf[35]) / param_.acc_factor.value;
    imu_msg.linear_acceleration.z = unpackBytesToFloat(buf[36], buf[37]) / param_.acc_factor.value;
    imu_msg.linear_acceleration_covariance = imu_raw_msg.linear_acceleration_covariance;

    imu_msg.angular_velocity.x = unpackBytesToFloat(buf[12], buf[13]) / param_.gyr_factor.value;
    imu_msg.angular_velocity.y = unpackBytesToFloat(buf[14], buf[15]) / param_.gyr_factor.value;
    imu_msg.angular_velocity.z = unpackBytesToFloat(buf[16], buf[17]) / param_.gyr_factor.value;
    imu_msg.angular_velocity_covariance = imu_raw_msg.angular_velocity_covariance;

    pub_imu_->publish(imu_msg);

    // Magnetometer
    mag_msg.header.stamp = node_->get_clock()->now().to_msg();
    mag_msg.header.frame_id = param_.frame_id.value;
    mag_msg.magnetic_field.x = unpackBytesToFloat(buf[6], buf[7]) / param_.mag_factor.value;
    mag_msg.magnetic_field.y = unpackBytesToFloat(buf[8], buf[9]) / param_.mag_factor.value;
    mag_msg.magnetic_field.z = unpackBytesToFloat(buf[10], buf[11]) / param_.mag_factor.value;
    mag_msg.magnetic_field_covariance[0] = param_.variance_mag.value[0];
    mag_msg.magnetic_field_covariance[4] = param_.variance_mag.value[1];
    mag_msg.magnetic_field_covariance[8] = param_.variance_mag.value[2];

    pub_mag_->publish(mag_msg);

    // Gravity
    grav_msg.x = unpackBytesToFloat(buf[38], buf[39]) / param_.grav_factor.value;
    grav_msg.y = unpackBytesToFloat(buf[40], buf[41]) / param_.grav_factor.value;
    grav_msg.z = unpackBytesToFloat(buf[42], buf[43]) / param_.grav_factor.value;

    pub_grav_->publish(grav_msg);

    // Temperature
    temp_msg.header.stamp = node_->get_clock()->now().to_msg();
    temp_msg.header.frame_id = param_.frame_id.value;
    temp_msg.temperature = static_cast<double>(buf[44]);
    pub_temp_->publish(temp_msg);
  }

  void get_calib_status() {
    auto calib = con_->receive(BNO055_CALIB_STAT_ADDR, 1);
    if (calib.empty()) return;
    uint8_t c = calib[0];
    int sys = (c >> 6) & 0x03;
    int gyro = (c >> 4) & 0x03;
    int accel = (c >> 2) & 0x03;
    int mag = c & 0x03;

    std_msgs::msg::String msg;
    msg.data = std::string("{\"sys\":") + std::to_string(sys) +
               ",\"gyro\":" + std::to_string(gyro) +
               ",\"accel\":" + std::to_string(accel) +
               ",\"mag\":" + std::to_string(mag) + "}";
    pub_calib_status_->publish(msg);
  }

  struct CalibData {
    int accel_x{}, accel_y{}, accel_z{};
    int accel_radius{};
    int mag_x{}, mag_y{}, mag_z{};
    int mag_radius{};
    int gyro_x{}, gyro_y{}, gyro_z{};
  };

  CalibData get_calib_data() {
    CalibData out;

    auto accel = con_->receive(ACCEL_OFFSET_X_LSB_ADDR, 6);
    if (accel.size() == 6) {
      out.accel_x = (accel[1] << 8) | accel[0];
      out.accel_y = (accel[3] << 8) | accel[2];
      out.accel_z = (accel[5] << 8) | accel[4];
    }

    auto accel_r = con_->receive(ACCEL_RADIUS_LSB_ADDR, 2);
    if (accel_r.size() == 2) out.accel_radius = (accel_r[1] << 8) | accel_r[0];

    auto mag = con_->receive(MAG_OFFSET_X_LSB_ADDR, 6);
    if (mag.size() == 6) {
      out.mag_x = (mag[1] << 8) | mag[0];
      out.mag_y = (mag[3] << 8) | mag[2];
      out.mag_z = (mag[5] << 8) | mag[4];
    }

    auto mag_r = con_->receive(MAG_RADIUS_LSB_ADDR, 2);
    if (mag_r.size() == 2) out.mag_radius = (mag_r[1] << 8) | mag_r[0];

    auto gyro = con_->receive(GYRO_OFFSET_X_LSB_ADDR, 6);
    if (gyro.size() == 6) {
      out.gyro_x = (gyro[1] << 8) | gyro[0];
      out.gyro_y = (gyro[3] << 8) | gyro[2];
      out.gyro_z = (gyro[5] << 8) | gyro[4];
    }

    return out;
  }

  void print_calib_data() {
    auto cd = get_calib_data();
    RCLCPP_INFO(node_->get_logger(), "\tAccel offsets (x y z): %d %d %d", cd.accel_x, cd.accel_y, cd.accel_z);
    RCLCPP_INFO(node_->get_logger(), "\tAccel radius: %d", cd.accel_radius);
    RCLCPP_INFO(node_->get_logger(), "\tMag offsets (x y z): %d %d %d", cd.mag_x, cd.mag_y, cd.mag_z);
    RCLCPP_INFO(node_->get_logger(), "\tMag radius: %d", cd.mag_radius);
    RCLCPP_INFO(node_->get_logger(), "\tGyro offsets (x y z): %d %d %d", cd.gyro_x, cd.gyro_y, cd.gyro_z);
  }

  bool set_calib_offsets(const Param<std::array<int16_t, 3>> &acc_offset,
                         const Param<std::array<int16_t, 3>> &mag_offset,
                         const Param<std::array<int16_t, 3>> &gyr_offset,
                         const Param<uint16_t> &mag_radius,
                         const Param<uint16_t> &acc_radius) {
    // Must switch to config mode to write out
    if (!con_->transmit(BNO055_OPR_MODE_ADDR, 1, std::vector<uint8_t>{OPERATION_MODE_CONFIG})) {
      RCLCPP_ERROR(node_->get_logger(), "Unable to set IMU into config mode");
    }
    std::this_thread::sleep_for(25ms);

    try {
      // ACCEL offsets
      con_->transmit(ACCEL_OFFSET_X_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(acc_offset.value[0] & 0xFF)});
      con_->transmit(ACCEL_OFFSET_X_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((acc_offset.value[0] >> 8) & 0xFF)});
      con_->transmit(ACCEL_OFFSET_Y_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(acc_offset.value[1] & 0xFF)});
      con_->transmit(ACCEL_OFFSET_Y_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((acc_offset.value[1] >> 8) & 0xFF)});
      con_->transmit(ACCEL_OFFSET_Z_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(acc_offset.value[2] & 0xFF)});
      con_->transmit(ACCEL_OFFSET_Z_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((acc_offset.value[2] >> 8) & 0xFF)});

      // ACCEL radius
      con_->transmit(ACCEL_RADIUS_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(acc_radius.value & 0xFF)});
      con_->transmit(ACCEL_RADIUS_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((acc_radius.value >> 8) & 0xFF)});

      // MAG offsets
      con_->transmit(MAG_OFFSET_X_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(mag_offset.value[0] & 0xFF)});
      con_->transmit(MAG_OFFSET_X_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((mag_offset.value[0] >> 8) & 0xFF)});
      con_->transmit(MAG_OFFSET_Y_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(mag_offset.value[1] & 0xFF)});
      con_->transmit(MAG_OFFSET_Y_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((magOffset.value[1] >> 8) & 0xFF)});
      con_->transmit(MAG_OFFSET_Z_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(mag_offset.value[2] & 0xFF)});
      con_->transmit(MAG_OFFSET_Z_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((mag_offset.value[2] >> 8) & 0xFF)});

      // MAG radius
      con_->transmit(MAG_RADIUS_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(mag_radius.value & 0xFF)});
      con_->transmit(MAG_RADIUS_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((mag_radius.value >> 8) & 0xFF)});

      // GYRO offsets
      con_->transmit(GYRO_OFFSET_X_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(gyr_offset.value[0] & 0xFF)});
      con_->transmit(GYRO_OFFSET_X_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((gyr_offset.value[0] >> 8) & 0xFF)});
      con_->transmit(GYRO_OFFSET_Y_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(gyr_offset.value[1] & 0xFF)});
      con_->transmit(GYRO_OFFSET_Y_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((gyr_offset.value[1] >> 8) & 0xFF)});
      con_->transmit(GYRO_OFFSET_Z_LSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>(gyr_offset.value[2] & 0xFF)});
      con_->transmit(GYRO_OFFSET_Z_MSB_ADDR, 1, std::vector<uint8_t>{static_cast<uint8_t>((gyr_offset.value[2] >> 8) & 0xFF)});

      return true;
    } catch (...) {
      return false;
    }
  }

  void calibration_request_callback(const std::shared_ptr<example_interfaces::srv::Trigger::Request> /*request*/,
                                    std::shared_ptr<example_interfaces::srv::Trigger::Response> response) {
    if (!con_->transmit(BNO055_OPR_MODE_ADDR, 1, std::vector<uint8_t>{OPERATION_MODE_CONFIG})) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set IMU into config mode.");
    }
    std::this_thread::sleep_for(25ms);
    auto cd = get_calib_data();
    if (!con_->transmit(BNO055_OPR_MODE_ADDR, 1, std::vector<uint8_t>{OPERATION_MODE_NDOF})) {
      RCLCPP_WARN(node_->get_logger(), "Unable to set IMU operation mode into operation mode.");
    }
    response->success = true;
    response->message = "{accel_offset:{x:" + std::to_string(cd.accel_x) + ",y:" + std::to_string(cd.accel_y) + ",z:" + std::to_string(cd.accel_z) +
                        "},accel_radius:" + std::to_string(cd.accel_radius) +
                        ",mag_offset:{x:" + std::to_string(cd.mag_x) + ",y:" + std::to_string(cd.mag_y) + ",z:" + std::to_string(cd.mag_z) +
                        "},mag_radius:" + std::to_string(cd.mag_radius) +
                        ",gyro_offset:{x:" + std::to_string(cd.gyro_x) + ",y:" + std::to_string(cd.gyro_y) + ",z:" + std::to_string(cd.gyro_z) + "}}";
  }

  static float unpackBytesToFloat(uint8_t start, uint8_t end) {
    int16_t value = static_cast<int16_t>((static_cast<int16_t>(end) << 8) | start);
    return static_cast<float>(value);
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<Connector> con_;
  NodeParameters param_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_grav_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_calib_status_;

  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr srv_;
};
