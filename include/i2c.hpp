#pragma once

#include <cstdint>
#include <vector>
#include <stdexcept>
#include <memory>

// Forward declaration for ROS2 node if needed
namespace rclcpp {
    class Node;
}

// Exception for transmission errors
class TransmissionException : public std::runtime_error {
public:
    explicit TransmissionException(const std::string& msg) : std::runtime_error(msg) {}
};

// Abstract base class for connectors
class Connector {
public:
    explicit Connector(std::shared_ptr<rclcpp::Node> node) : node_(node) {}
    virtual ~Connector() = default;

    // Not protocol-specific
    virtual std::vector<uint8_t> receive(uint8_t reg_addr, size_t length) {
        return read(reg_addr, length);
    }
    virtual bool transmit(uint8_t reg_addr, size_t length, const std::vector<uint8_t>& data) {
        return write(reg_addr, length, data);
    }

protected:
    std::shared_ptr<rclcpp::Node> node_;
    virtual std::vector<uint8_t> read(uint8_t reg_addr, size_t length) = 0;
    virtual bool write(uint8_t reg_addr, size_t length, const std::vector<uint8_t>& data) = 0;
};

// I2C connector implementation
class I2C : public Connector {
public:
    static constexpr const char* CONNECTIONTYPE_I2C = "i2c";

    I2C(std::shared_ptr<rclcpp::Node> node, int i2c_bus = 0, uint8_t i2c_addr = 0x28);
    virtual ~I2C();

    void connect();

protected:
    std::vector<uint8_t> read(uint8_t reg_addr, size_t length) override;
    bool write(uint8_t reg_addr, size_t length, const std::vector<uint8_t>& data) override;

private:
    int bus_fd_;
    uint8_t address_;
    int i2c_bus_;
    // Helper for opening/closing bus, etc.
};
