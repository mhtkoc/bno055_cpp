#include "i2c.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <iostream>

I2C::I2C(std::shared_ptr<rclcpp::Node> node, int i2c_bus, uint8_t i2c_addr)
    : Connector(node), bus_fd_(-1), address_(i2c_addr), i2c_bus_(i2c_bus) {}

I2C::~I2C() {
    if (bus_fd_ >= 0) {
        close(bus_fd_);
    }
}

void I2C::connect() {
    char filename[32];
    snprintf(filename, sizeof(filename), "/dev/i2c-%d", i2c_bus_);
    bus_fd_ = open(filename, O_RDWR);
    if (bus_fd_ < 0) {
        throw TransmissionException("Failed to open I2C bus");
    }
    if (ioctl(bus_fd_, I2C_SLAVE, address_) < 0) {
        close(bus_fd_);
        bus_fd_ = -1;
        throw TransmissionException("Failed to set I2C address");
    }
}

std::vector<uint8_t> I2C::read(uint8_t reg_addr, size_t length) {
    if (bus_fd_ < 0) throw TransmissionException("I2C bus not open");
    // Write register address
    if (::write(bus_fd_, &reg_addr, 1) != 1) {
        throw TransmissionException("Failed to write register address");
    }
    std::vector<uint8_t> buffer(length);
    ssize_t bytesRead = ::read(bus_fd_, buffer.data(), length);
    if (bytesRead != (ssize_t)length) {
        throw TransmissionException("Failed to read from I2C device");
    }
    return buffer;
}

bool I2C::write(uint8_t reg_addr, size_t length, const std::vector<uint8_t>& data) {
    if (bus_fd_ < 0) throw TransmissionException("I2C bus not open");
    std::vector<uint8_t> buffer(length + 1);
    buffer[0] = reg_addr;
    if (data.size() < length) {
        throw TransmissionException("Data size less than length");
    }
    std::memcpy(buffer.data() + 1, data.data(), length);
    ssize_t bytesWritten = ::write(bus_fd_, buffer.data(), length + 1);
    if (bytesWritten != (ssize_t)(length + 1)) {
        throw TransmissionException("Failed to write to I2C device");
    }
    return true;
}
