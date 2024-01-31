#pragma once

#include <cstdint>
#include <optional>

namespace JetBotControl {

class I2CDevice {
 public:
  I2CDevice();
  ~I2CDevice();
  bool tryEnableMotor(uint8_t pin);
  bool trySetDutyCycle(uint8_t pin, uint16_t duty_cycle);

 private:
  bool trySelectDevice();
  bool tryWriteReg(uint8_t reg, uint8_t data);
  std::optional<uint8_t> tryReadReg(uint8_t reg);
  bool trySetClock();
  bool tryReset();
  int i2c_fd_;
  uint8_t buf_[10];
};

}  // namespace JetBotControl
