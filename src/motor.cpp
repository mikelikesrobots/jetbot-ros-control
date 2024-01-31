#include "jetbot_control/motor.hpp"

using namespace JetBotControl;

#include <string>

Motor::Motor(I2CDevicePtr i2c, MotorPins pins, uint32_t motor_number)
    : i2c_{i2c}, pins_{pins}, motor_number_{motor_number} {
  u8 enable_pin = std::get<0>(pins_);
  if (!i2c_->tryEnableMotor(enable_pin)) {
    std::string error =
        "Failed to enable motor " + std::to_string(motor_number) + "!";
    std::__throw_runtime_error(error.c_str());
  }
}

Motor::~Motor() {
  trySetSpinning(false);
}

bool Motor::trySetSpinning(bool spinning) {
  u8 pos_pin = std::get<1>(pins_);
  u8 neg_pin = std::get<2>(pins_);
  if (spinning) {
    if (!i2c_->trySetDutyCycle(pos_pin, 0xFFFF)) {
      return false;
    }
    if (!i2c_->trySetDutyCycle(neg_pin, 0)) {
      return false;
    }
  } else {
    if (!i2c_->trySetDutyCycle(pos_pin, 0xFFFF)) {
      return false;
    }
    if (!i2c_->trySetDutyCycle(neg_pin, 0xFFFF)) {
      return false;
    }
  }
  return true;
}
