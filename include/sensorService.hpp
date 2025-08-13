#include <memory>
#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <chrono>

#pragma once
#include <memory>
#include <vector>
#include <array>
#include <string>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "registers.hpp"
#include "i2c.hpp"

template <typename T>
struct Param {
  T value{};
};

struct NodeParameters {
  Param<std::string> ros_topic_prefix;
  Param<std::string> placement_axis_remap;
  Param<bool> set_offsets;
  Param<uint8_t> operation_mode;
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
                const NodeParameters &param,
                bool publish_raw_imu = false);
  void configure();
  void get_sensor_data();
  void get_calib_status();
  struct CalibData {
    int accel_x{}, accel_y{}, accel_z{};
    int accel_radius{};
    int mag_x{}, mag_y{}, mag_z{};
    int mag_radius{};
    int gyro_x{}, gyro_y{}, gyro_z{};
  };
  CalibData get_calib_data();
  void print_calib_data();
  bool set_calib_offsets(const Param<std::array<int16_t, 3>> &acc_offset,
                         const Param<std::array<int16_t, 3>> &mag_offset,
                         const Param<std::array<int16_t, 3>> &gyr_offset,
                         const Param<uint16_t> &mag_radius,
                         const Param<uint16_t> &acc_radius);
  void calibration_request_callback(const std::shared_ptr<example_interfaces::srv::Trigger::Request> request,
                                    std::shared_ptr<example_interfaces::srv::Trigger::Response> response);
  static float unpackBytesToFloat(uint8_t start, uint8_t end);
private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<Connector> con_;
  NodeParameters param_;
  bool publish_raw_imu_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_raw_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_grav_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_calib_status_;
  rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr srv_;
};


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
