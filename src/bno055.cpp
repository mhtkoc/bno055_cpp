
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "../include/i2c.hpp"
#include "../include/exceptions.hpp"
#include "../include/sensorService.hpp"

using namespace std::chrono_literals;



class Bno055Node : public rclcpp::Node {
public:
	Bno055Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("bno055", options) {
		// Declare and get all parameters from YAML or CLI
		param_ = NodeParameters{};
		declare_parameter<std::string>("ros_topic_prefix", "bno055/");
		declare_parameter<std::string>("placement_axis_remap", "P0");
		declare_parameter<bool>("set_offsets", false);
		declare_parameter<uint8_t>("operation_mode", 0x0C);
		declare_parameter<std::string>("frame_id", "bno055");
		declare_parameter<std::vector<double>>("variance_orientation", {0.0159, 0.0159, 0.0159});
		declare_parameter<std::vector<double>>("variance_acc", {0.017, 0.017, 0.017});
		declare_parameter<std::vector<double>>("variance_angular_vel", {0.04, 0.04, 0.04});
		declare_parameter<std::vector<double>>("variance_mag", {0.0, 0.0, 0.0});
		declare_parameter<double>("acc_factor", 100.0);
		declare_parameter<double>("gyr_factor", 16.0);
		declare_parameter<double>("mag_factor", 16.0);
		declare_parameter<double>("grav_factor", 100.0);
		declare_parameter<std::vector<int64_t>>("offset_acc", {0, 0, 0});
		declare_parameter<std::vector<int64_t>>("offset_mag", {0, 0, 0});
		declare_parameter<std::vector<int64_t>>("offset_gyr", {0, 0, 0});
		declare_parameter<uint16_t>("radius_mag", 0);
		declare_parameter<uint16_t>("radius_acc", 1000);
		declare_parameter<int>("i2c_bus", 1);
		declare_parameter<int>("i2c_addr", 0x28);
		declare_parameter<bool>("publish_raw_imu", false);
		declare_parameter<double>("data_query_frequency", 100.0);
		declare_parameter<double>("calib_status_frequency", 0.1);

		get_parameter("ros_topic_prefix", param_.ros_topic_prefix.value);
		get_parameter("placement_axis_remap", param_.placement_axis_remap.value);
		get_parameter("set_offsets", param_.set_offsets.value);
		get_parameter("operation_mode", param_.operation_mode.value);
		get_parameter("frame_id", param_.frame_id.value);
		std::vector<double> v;
		get_parameter("variance_orientation", v); for (size_t i=0; i<3 && i<v.size(); ++i) param_.variance_orientation.value[i]=v[i];
		get_parameter("variance_acc", v); for (size_t i=0; i<3 && i<v.size(); ++i) param_.variance_acc.value[i]=v[i];
		get_parameter("variance_angular_vel", v); for (size_t i=0 && i<v.size(); ++i) param_.variance_angular_vel.value[i]=v[i];
		get_parameter("variance_mag", v); for (size_t i=0; i<3 && i<v.size(); ++i) param_.variance_mag.value[i]=v[i];
		get_parameter("acc_factor", param_.acc_factor.value);
		get_parameter("gyr_factor", param_.gyr_factor.value);
		get_parameter("mag_factor", param_.mag_factor.value);
		get_parameter("grav_factor", param_.grav_factor.value);
		std::vector<int64_t> vi;
		get_parameter("offset_acc", vi); for (size_t i=0; i<3 && i<vi.size(); ++i) param_.offset_acc.value[i]=static_cast<int16_t>(vi[i]);
		get_parameter("offset_mag", vi); for (size_t i=0; i<3 && i<vi.size(); ++i) param_.offset_mag.value[i]=static_cast<int16_t>(vi[i]);
		get_parameter("offset_gyr", vi); for (size_t i=0; i<3 && i<vi.size(); ++i) param_.offset_gyr.value[i]=static_cast<int16_t>(vi[i]);
		get_parameter("radius_mag", param_.radius_mag.value);
		get_parameter("radius_acc", param_.radius_acc.value);
		int i2c_bus = 1;
		int i2c_addr = 0x28;
		get_parameter("i2c_bus", i2c_bus);
		get_parameter("i2c_addr", i2c_addr);
		bool publish_raw_imu = false;
		get_parameter("publish_raw_imu", publish_raw_imu);
		get_parameter("data_query_frequency", data_query_frequency_);
		get_parameter("calib_status_frequency", calib_status_frequency_);

		connector_ = std::make_shared<I2C>(shared_from_this(), i2c_bus, static_cast<uint8_t>(i2c_addr));
		connector_->connect();
		sensor_ = std::make_shared<SensorService>(shared_from_this(), connector_, param_, publish_raw_imu);
		sensor_->configure();
	}

	std::shared_ptr<SensorService> sensor_;
	NodeParameters param_;
	std::shared_ptr<I2C> connector_;
	double data_query_frequency_ = 100.0;
	double calib_status_frequency_ = 0.1;
};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Bno055Node>();
	std::mutex lock;

	auto read_data = [&]() {
		if (lock.try_lock()) {
			try {
				node->sensor_->get_sensor_data();
			} catch (const BusOverRunException &e) {
				// Data not available yet, skip
			} catch (const std::exception &e) {
				RCLCPP_WARN(node->get_logger(), "Receiving sensor data failed with %s: '%s'", typeid(e).name(), e.what());
			}
			lock.unlock();
		} else {
			RCLCPP_WARN(node->get_logger(), "Message communication in progress - skipping query cycle");
		}
	};

	auto log_calibration_status = [&]() {
		if (lock.try_lock()) {
			try {
				node->sensor_->get_calib_status();
			} catch (const std::exception &e) {
				RCLCPP_WARN(node->get_logger(), "Receiving calibration status failed with %s: '%s'", typeid(e).name(), e.what());
			}
			lock.unlock();
		} else {
			RCLCPP_WARN(node->get_logger(), "Message communication in progress - skipping query cycle");
		}
	};

	auto timer1 = node->create_wall_timer(std::chrono::duration<double>(1.0 / node->data_query_frequency_), read_data);
	auto timer2 = node->create_wall_timer(std::chrono::duration<double>(1.0 / node->calib_status_frequency_), log_calibration_status);

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}


