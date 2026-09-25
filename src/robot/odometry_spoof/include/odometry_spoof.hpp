#ifndef ODOMETRY_SPOOF_NODE_HPP_
#define ODOMETRY_SPOOF_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdometrySpoofNode : public rclcpp::Node
{
  public:
    OdometrySpoofNode();

  private:
    void odomCallback(
      const nav_msgs::msg::Odometry::SharedPtr msg);

    // Receives the actual odometry produced by Gazebo
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gazebo_odom_sub_;

    // Publishes the odometry topic provided to the assignment nodes
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

#endif
