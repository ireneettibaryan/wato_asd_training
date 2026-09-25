#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    // Receives the latest local costmap
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

    // Receives the robot's current position and orientation
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publishes the global map
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // Limits how often the global map is updated
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::OccupancyGrid global_map_;

    nav_msgs::msg::Odometry latest_odom_;

    bool have_costmap_;
    bool have_odom_;
    bool have_fused_once_;

    double last_update_x_;
    double last_update_y_;

    const double update_distance_ = 1.5;

    void costmapCallback(
      const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void odomCallback(
      const nav_msgs::msg::Odometry::SharedPtr msg);

    void updateMap();

    void integrateCostmap();

    double getYaw(
      const geometry_msgs::msg::Quaternion & q);
};

#endif