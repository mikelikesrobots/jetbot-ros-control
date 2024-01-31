#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "jetbot_control/i2c_device.hpp"
#include "jetbot_control/motor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class JetBotControlNode : public rclcpp::Node {
 public:
  JetBotControlNode() : Node("jetbot_control_node"), spinning_(false) {
    device_ptr_ = std::make_shared<JetBotControl::I2CDevice>();
    motor_1_ = JetBotControl::Motor(device_ptr_, std::make_tuple(8, 9, 10), 1);
    motor_2_ =
        JetBotControl::Motor(device_ptr_, std::make_tuple(13, 11, 12), 2);

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&JetBotControlNode::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Motors are spinning: " + std::to_string(spinning_);
    motor_1_.trySetSpinning(spinning_);
    motor_2_.trySetSpinning(spinning_);
    RCLCPP_INFO(this->get_logger(), message.data.c_str());
    publisher_->publish(message);

    // Start/stop motors every callback
    spinning_ = !spinning_;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  bool spinning_;

  std::shared_ptr<JetBotControl::I2CDevice> device_ptr_;
  JetBotControl::Motor motor_1_;
  JetBotControl::Motor motor_2_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JetBotControlNode>());
  rclcpp::shutdown();
  return 0;
}
