#include "rclcpp/rclcpp.hpp"

class DroneController : public rclcpp::Node
{
public:
  DroneController() : Node("drone_controller")
  {
    // Create a timer to periodically print a message
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        RCLCPP_INFO(this->get_logger(), "Drone controller is running");
      });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneController>());
  rclcpp::shutdown();
  return 0;
}