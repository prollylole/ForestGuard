/**
 * @brief Visualization node for displaying detected objects in RViz
 * @details This class handles:
 *  - Subscribing to human and door detection topics
 *  - Converting detections to RViz markers
 *  - Publishing markers for visualization
 *  - Maintaining unique IDs for different marker types
 */

 #include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <string>
#include <sstream>
#include <chrono>

#include "husky_controller/vis_marker.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

VisualisationMarker::VisualisationMarker() :
  Node("visualisation_marker"),
  marker_counter_human(0)
{
  // SUBSCRIBE TO /drone/detected_humans, EXPECT MESSAGE geometry_msgs::msg::PoseArray
  // BIND WIRES std_msgs::msg::String FROM /drone/detected_humans TO humanCallback
  human_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/X4/detected_humans", 10, std::bind(&VisualisationMarker::humanCallback, this, _1));

  // MAKE A SUB TOPIC /drone/detected_doors TO TELL MARKER WHERE TO FIND DOORS
  door_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/X4/detected_doors", 10, std::bind(&VisualisationMarker::doorCallback, this, _1));
  
  PubVisual_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker",3);  

  timer_ = this->create_wall_timer(50ms, std::bind(&VisualisationMarker::publishMarker, this));
}

/**
 * @brief Destructor ensures proper cleanup of resources
 */
VisualisationMarker::~VisualisationMarker()
{
  // Clear stored detections
  detected_humans.clear();
  detected_doors.clear();

  // Reset smart pointers
  human_sub_.reset();
  door_sub_.reset();
  PubVisual_.reset();
  timer_.reset();
}

// The subscription to /drone/detected_doors (and /drone/detected_humans) belongs in the VisualisationMarker node. 
// This node is purely responsible for transforming PoseArrays into RViz markers. 
// let Quadcopter only publish those PoseArrays, and let VisualisationMarker only subscribe.

// WHEN std_msgs::msg::String ARRIVES, ROS DESERIALISE INTO std_msgs::msg::String::UniquePtr
// THEN INVOKES topic_callback

/**
 * @brief Callback for processing human detection messages
 * @param msg PoseArray containing detected human positions
 */
void VisualisationMarker::humanCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg) {
  // clear the detected_humans vector
  detected_humans.clear();

  // for each pose in pose array, copy the data to human_point
  for (const auto& pose : msg->poses) {
    geometry_msgs::msg::Point human_point;
    human_point.x = pose.position.x;
    human_point.y = pose.position.y;
    human_point.z = pose.position.z; 

    // push back the human_point of each pose into a vector
    detected_humans.push_back(human_point);
  }
}

/**
 * @brief Callback for processing door detection messages
 * @param msg PoseArray containing detected door positions
 */
void VisualisationMarker::doorCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg) {
  // clear the detected_humans vector
  detected_doors.clear();
  std::cout << "Received doors" << std::endl;

  // for each pose in pose array, copy the data to human_point
  for (const auto& pose : msg->poses) {
    geometry_msgs::msg::Point door_point;
    door_point.x = pose.position.x;
    door_point.y = pose.position.y;
    door_point.z = pose.position.z; 

    // push back the human_point of each pose into a vector
    detected_doors.push_back(door_point);
  }
}

/**
 * @brief Timer callback to publish visualization markers
 */
// publish markers to the /visualization_marker topic in form of a MarkerArray
void VisualisationMarker::publishMarker() {
  // set stamp as current time
  auto stamp = this->now();

  visualization_msgs::msg::MarkerArray markers;

  // call humanMarkers to populate markers with human markers
  humanMarkers(markers, stamp);
  // call doorMarkers to populate markers with door markers
  doorMarkers(markers, stamp);
  // publish the markers to the /visualization_marker topic
  PubVisual_->publish(markers);
}

/**
 * @brief Creates visualization markers for detected humans
 * @param markers MarkerArray to add human markers to
 * @param stamp Timestamp for the markers
 */
// publishes produceMarker with a set of time
void VisualisationMarker::humanMarkers(visualization_msgs::msg::MarkerArray& markers, rclcpp::Time stamp) {
  // for each human inside detected_humans, create a marker
  for (const auto& human : detected_humans) {
    visualization_msgs::msg::Marker marker;

    // Set the frame ID and time stamp.
    marker.header.frame_id = "world";
    marker.header.stamp = stamp;

    // namespace is person
    marker.ns = "person";
    marker.id = marker_counter_human++;

    // human marker type is cylinder, add action
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // set position and orientation
    marker.pose.position.x = human.x;
    marker.pose.position.y = human.y;
    marker.pose.position.z = human.z; 
    marker.pose.orientation.w = 1.0; // no rotation

    // DOUBLE CHECK THIS IS RADIUS NOT DIAMETER
    // radius is 0.2, height is 2.0m
    marker.scale.x = 0.2; // radius
    marker.scale.y = 0.2; // radius
    marker.scale.z = 2.0; // height

    // set color to green, fully opaque
    marker.color.r = 0.0f; // red
    marker.color.g = 1.0f; // no green
    marker.color.b = 0.0f; // no blue
    marker.color.a = 1.0f; // fully opaque

    //We set lifetime (it will dissapear in this many seconds)
    marker.lifetime = rclcpp::Duration(30, 0); // lifetime of the marker

    std::cout << "publishing humans at: " << human.x << human.y << human.z << std::endl;
    // Add the marker to the array
    markers.markers.push_back(marker);
  }
}

/**
 * @brief Creates visualization markers for detected doors
 * @param markers MarkerArray to add door markers to
 * @param stamp Timestamp for the markers
 */
void VisualisationMarker::doorMarkers(visualization_msgs::msg::MarkerArray& markers, rclcpp::Time stamp) {
  // for each door inside detected_doors, create a marker
  for (const auto& door : detected_doors) {
    visualization_msgs::msg::Marker marker;

    // Set the frame ID and time stamp.
    marker.header.frame_id = "world";
    marker.header.stamp = stamp;

    // namespace is door
    marker.ns = "door";
    marker.id = marker_counter_door++;

    // door marker type is cube, add action
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // set position and orientation
    marker.pose.position.x = door.x;
    marker.pose.position.y = door.y;
    marker.pose.position.z = door.z; 
    marker.pose.orientation.w = 1.0; // no rotation

    // DOUBLE CHECK THE SIZE OF THE DOOR
    // x, y = 2m , height = 2.5m, depth = 0.2m
    marker.scale.x = 2.0; // width
    marker.scale.y = 0.2; // depth
    marker.scale.z = 2.5; // height

    // set color to blue, fully opaque
    marker.color.r = 0.0f; // no red
    marker.color.g = 0.0f; // no green
    marker.color.b = 1.0f; // blue
    marker.color.a = 1.0f; // fully opaque

    //We set lifetime (it will dissapear in this many seconds)
    marker.lifetime = rclcpp::Duration(30, 0); // lifetime of the marker
    std::cout << "publishing door" << std::endl;

    // Add the marker to the array
    markers.markers.push_back(marker);
  }
}