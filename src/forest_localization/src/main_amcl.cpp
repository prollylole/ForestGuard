#include "forest_localization/amcl_localization.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmclLocalization>());
  rclcpp::shutdown();
  return 0;
}
