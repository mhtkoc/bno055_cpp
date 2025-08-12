#pragma once

#include <stdexcept>
#include <string>

// Exception thrown in case of failing transmissions between host and BNO055 sensor device.
class TransmissionException : public std::runtime_error {
public:
    explicit TransmissionException(const std::string& msg)
        : std::runtime_error(msg) {}
};

// Exception thrown when BNO055 sensor device data fusion was not ready.
// See Github Issue #5 (https://github.com/flynneva/bno055/issues/5)
class BusOverRunException : public TransmissionException {
public:
    explicit BusOverRunException(const std::string& msg)
        : TransmissionException(msg) {}
};
