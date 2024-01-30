#pragma once

#include <memory>
#include <tuple>

#include "jetbot_control/i2c_device.hpp"

using u8 = uint8_t;
using I2CDevicePtr = std::shared_ptr<JetBotControl::I2CDevice>;
using MotorPins = std::tuple<u8, u8, u8>;

namespace JetBotControl {
class Motor {
 public:
  Motor() = default;
  Motor(I2CDevicePtr i2c, MotorPins pins, uint32_t motor_number);
  ~Motor();
  bool trySetSpinning(bool spinning);

 private:
  I2CDevicePtr i2c_;
  MotorPins pins_;
  uint32_t motor_number_;
};
}  // namespace JetBotControl
