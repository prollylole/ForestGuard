#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "controller_pkg/vis_marker.h"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform initialisation of node
   * Thereafter we create a share pointer to the Sample class and call the spin method which kicks of the node
   * Finally we shutdown ros
   */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisualisationMarker>());
  rclcpp::shutdown();
 
  return 0;
}