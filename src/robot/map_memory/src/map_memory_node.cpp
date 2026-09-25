#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(robot::MapMemoryCore(this->get_logger())),
  have_costmap_(false),
  have_odom_(false),
  have_fused_once_(false),
  last_update_x_(0.0),
  last_update_y_(0.0)
{
  // Subscribe to the local costmap
  costmap_sub_ =
    this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap",
      10,
      std::bind(
        &MapMemoryNode::costmapCallback,
        this,
        std::placeholders::_1
      )
    );

  // Subscribe to robot odometry
  odom_sub_ =
    this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered",
      10,
      std::bind(
        &MapMemoryNode::odomCallback,
        this,
        std::placeholders::_1
      )
    );

  // Publish the global map
  map_pub_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map",
      10
    );

  // Create a larger, lower-resolution global map
  global_map_.header.frame_id = "robot/odom";

  global_map_.info.resolution = 0.2;

  global_map_.info.width = 200;
  global_map_.info.height = 200;

  global_map_.info.origin.position.x = -20.0;
  global_map_.info.origin.position.y = -20.0;
  global_map_.info.origin.position.z = 0.0;

  global_map_.info.origin.orientation.x = 0.0;
  global_map_.info.origin.orientation.y = 0.0;
  global_map_.info.origin.orientation.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  // -1 means the global cell has not been observed yet
  global_map_.data.assign(
    global_map_.info.width * global_map_.info.height,
    -1
  );

  // Publish an initial map immediately
  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);

  // Check once per second whether the map needs updating
  timer_ =
    this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapMemoryNode::updateMap, this)
    );
}


// Store the newest local costmap
void MapMemoryNode::costmapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = *msg;
  have_costmap_ = true;
}


// Store the newest robot odometry
void MapMemoryNode::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
  have_odom_ = true;
}


// Convert quaternion orientation into yaw
double MapMemoryNode::getYaw(
  const geometry_msgs::msg::Quaternion & q)
{
  double siny_cosp =
    2.0 * (q.w * q.z + q.x * q.y);

  double cosy_cosp =
    1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  return std::atan2(siny_cosp, cosy_cosp);
}


// Timer callback that decides whether the global map should update
void MapMemoryNode::updateMap()
{
  if (!have_costmap_ || !have_odom_) {
    return;
  }

  double current_x =
    latest_odom_.pose.pose.position.x;

  double current_y =
    latest_odom_.pose.pose.position.y;

  double dx = current_x - last_update_x_;
  double dy = current_y - last_update_y_;

  double distance =
    std::sqrt(dx * dx + dy * dy);

  // Fuse immediately the first time,
  // then only after the robot moves far enough
  if (!have_fused_once_ ||
      distance >= update_distance_) {

    integrateCostmap();

    last_update_x_ = current_x;
    last_update_y_ = current_y;

    have_fused_once_ = true;
  }

  // Keep publishing the latest global map
  global_map_.header.stamp =
    this->get_clock()->now();

  map_pub_->publish(global_map_);
}


// Transform the local costmap into the global map
void MapMemoryNode::integrateCostmap()
{
  double robot_x =
    latest_odom_.pose.pose.position.x;

  double robot_y =
    latest_odom_.pose.pose.position.y;

  double robot_yaw =
    getYaw(latest_odom_.pose.pose.orientation);

  double cos_yaw = std::cos(robot_yaw);
  double sin_yaw = std::sin(robot_yaw);


  for (unsigned int local_y = 0;
       local_y < latest_costmap_.info.height;
       ++local_y) {

    for (unsigned int local_x = 0;
         local_x < latest_costmap_.info.width;
         ++local_x) {

      int local_index =
        local_y * latest_costmap_.info.width +
        local_x;

      int cell_value =
        latest_costmap_.data[local_index];


      // Skip unknown cells
      if (cell_value < 0) {
        continue;
      }


      // Position of the local cell in the robot/lidar frame
      double local_world_x =
        latest_costmap_.info.origin.position.x +
        (static_cast<double>(local_x) + 0.5) *
        latest_costmap_.info.resolution;

      double local_world_y =
        latest_costmap_.info.origin.position.y +
        (static_cast<double>(local_y) + 0.5) *
        latest_costmap_.info.resolution;


      // Rotate and translate the local cell into the global frame
      double global_x =
        robot_x +
        local_world_x * cos_yaw -
        local_world_y * sin_yaw;

      double global_y =
        robot_y +
        local_world_x * sin_yaw +
        local_world_y * cos_yaw;


      // Convert global coordinates into global grid indices
      int global_grid_x =
        static_cast<int>(
          (global_x -
           global_map_.info.origin.position.x) /
          global_map_.info.resolution
        );

      int global_grid_y =
        static_cast<int>(
          (global_y -
           global_map_.info.origin.position.y) /
          global_map_.info.resolution
        );


      // Ignore points outside the global map
      if (global_grid_x < 0 ||
          global_grid_x >=
            static_cast<int>(global_map_.info.width) ||
          global_grid_y < 0 ||
          global_grid_y >=
            static_cast<int>(global_map_.info.height)) {

        continue;
      }


      int global_index =
        global_grid_y * global_map_.info.width +
        global_grid_x;

      // New known data replaces the old global-map value
      global_map_.data[global_index] = cell_value;
    }
  }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<MapMemoryNode>()
  );

  rclcpp::shutdown();

  return 0;
}
