#include <memory>

#include "odometry_spoof.hpp"

OdometrySpoofNode::OdometrySpoofNode()
: Node("odometry_spoof")
{
    // Publish the odometry topic expected by the assignment.
    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10);

    // Read the actual moving robot odometry provided by Gazebo.
    gazebo_odom_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/robot/odometry",
            10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                this->odomCallback(msg);
            });
}

void OdometrySpoofNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    nav_msgs::msg::Odometry odom = *msg;

    // Republish Gazebo's current robot pose for the assignment nodes.
    odom_pub_->publish(odom);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometrySpoofNode>());
    rclcpp::shutdown();

    return 0;
}
