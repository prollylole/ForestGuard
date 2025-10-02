#ifndef VIS_MARKER_H
#define VIS_MARKER_H

/**
 * @brief Visualization node for displaying detected objects in RViz
 * @details This class handles:
 *  - Subscribing to human and door detection topics
 *  - Converting detections to RViz markers
 *  - Publishing markers for visualization
 *  - Maintaining unique IDs for different marker types
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

/**
 * @brief Class for visualizing detected humans and doors in RViz
 * @details Subscribes to PoseArray messages containing human and door positions
 * and publishes visualization markers for display in RViz
 */
class VisualisationMarker : public rclcpp::Node
{
public:
    /**
     * @brief Constructor initializes ROS node and sets up publishers/subscribers
     */
    VisualisationMarker();

    /**
     * @brief Destructor ensures proper cleanup of resources
     */
    ~VisualisationMarker();


private:
    /**
     * @brief Callback for processing human detection messages
     * @param msg PoseArray containing detected human positions
     */
    void humanCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    /**
     * @brief Callback for processing door detection messages
     * @param msg PoseArray containing detected door positions
     */
    void doorCallback(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

    /**
     * @brief Timer callback to publish visualization markers
     */
    void publishMarker();

    /**
     * @brief Creates visualization markers for detected humans
     * @param markers MarkerArray to add human markers to
     * @param stamp Timestamp for the markers
     */
    void humanMarkers(visualization_msgs::msg::MarkerArray& markers, rclcpp::Time stamp);

    /**
     * @brief Creates visualization markers for detected doors
     * @param markers MarkerArray to add door markers to
     * @param stamp Timestamp for the markers
     */
    void doorMarkers(visualization_msgs::msg::MarkerArray& markers, rclcpp::Time stamp);
    
    // ROS communication
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr human_sub_;///< Subscriber for human detections
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr door_sub_;///< Subscriber for door detections
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr PubVisual_;///< Publisher for visualization markers
    rclcpp::TimerBase::SharedPtr timer_;///< Timer for periodic marker publishing

    // data storage members
    std::vector<geometry_msgs::msg::Point> detected_humans;///< Current human detection positions
    std::vector<geometry_msgs::msg::Point> detected_doors;///< Current door detection positions

    // data counters
    unsigned int marker_counter_human;///< Unique ID counter for human markers
    unsigned int marker_counter_door;///< Unique ID counter for door markers
};

#endif // VIS_MARKER_H