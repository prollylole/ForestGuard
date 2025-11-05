#include "forestguard_localisation/amcl_localisation.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Amcllocalisation>());
  rclcpp::shutdown();
  return 0;
}
