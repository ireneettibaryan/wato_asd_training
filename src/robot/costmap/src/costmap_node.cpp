#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "costmap_node.hpp"

CostmapNode::CostmapNode()
: Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  // Publishes the generated occupancy grid
  costmap_pub_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // Receives lidar scans from the simulated robot
  lidar_sub_ =
    this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar",
      10,
      std::bind(
        &CostmapNode::lidarCallback,
        this,
        std::placeholders::_1
      )
    );
}


// Converts each lidar scan into a local occupancy grid
void CostmapNode::lidarCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // Costmap parameters
  const double resolution = 0.1;

  const int width = 200;
  const int height = 200;

  const double origin_x =
    -1.0 * width * resolution / 2.0;

  const double origin_y =
    -1.0 * height * resolution / 2.0;

  const double inflation_radius = 1.0;

  const int max_cost = 100;


  // Creates the occupancy grid message
  nav_msgs::msg::OccupancyGrid grid;

  grid.header.stamp = this->get_clock()->now();

  grid.header.frame_id = scan->header.frame_id;

  grid.info.resolution = resolution;

  grid.info.width = width;

  grid.info.height = height;

  grid.info.origin.position.x = origin_x;

  grid.info.origin.position.y = origin_y;

  grid.info.origin.position.z = 0.0;

  grid.info.origin.orientation.x = 0.0;

  grid.info.origin.orientation.y = 0.0;

  grid.info.origin.orientation.z = 0.0;

  grid.info.origin.orientation.w = 1.0;


  // Initializes every cell as free space
  grid.data.assign(width * height, 0);


  // Stores obstacle cells so that they can be inflated afterwards
  std::vector<std::pair<int, int>> obstacles;


  // Goes through every lidar range measurement
  for (size_t i = 0; i < scan->ranges.size(); ++i) {

    double range = scan->ranges[i];

    // Ignores invalid lidar readings
    if (!std::isfinite(range) ||
        range <= scan->range_min ||
        range >= scan->range_max) {

      continue;
    }


    // Calculates the angle belonging to this lidar measurement
    double angle =
      scan->angle_min +
      (static_cast<double>(i) * scan->angle_increment);


    // Converts polar lidar coordinates into Cartesian coordinates
    double x = range * std::cos(angle);

    double y = range * std::sin(angle);


    // Converts Cartesian coordinates into grid indices
    int grid_x =
      static_cast<int>((x - origin_x) / resolution);

    int grid_y =
      static_cast<int>((y - origin_y) / resolution);


    // Makes sure the obstacle lies inside the costmap
    if (grid_x >= 0 &&
        grid_x < width &&
        grid_y >= 0 &&
        grid_y < height) {

      int index =
        grid_y * width + grid_x;

      // Marks the obstacle itself with the highest cost
      grid.data[index] = max_cost;

      obstacles.push_back({grid_x, grid_y});
    }
  }


  // Number of cells covered by the inflation radius
  int inflation_cells =
    static_cast<int>(
      std::ceil(inflation_radius / resolution)
    );


  // Inflates every obstacle
  for (const auto & obstacle : obstacles) {

    int obstacle_x = obstacle.first;

    int obstacle_y = obstacle.second;


    for (int dx = -inflation_cells;
         dx <= inflation_cells;
         ++dx) {

      for (int dy = -inflation_cells;
           dy <= inflation_cells;
           ++dy) {

        int new_x = obstacle_x + dx;

        int new_y = obstacle_y + dy;


        // Skips cells outside the costmap
        if (new_x < 0 ||
            new_x >= width ||
            new_y < 0 ||
            new_y >= height) {

          continue;
        }


        // Calculates the real-world distance from the obstacle
        double distance =
          std::sqrt(
            static_cast<double>(dx * dx + dy * dy)
          ) * resolution;


        // Only cells within the inflation radius receive a cost
        if (distance <= inflation_radius) {

          int cost =
            static_cast<int>(
              max_cost *
              (1.0 - distance / inflation_radius)
            );


          int index =
            new_y * width + new_x;


          // Keeps whichever cost is larger
          grid.data[index] =
            std::max(
              static_cast<int>(grid.data[index]),
              cost
            );
        }
      }
    }
  }


  // Publishes the finished costmap
  costmap_pub_->publish(grid);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<CostmapNode>()
  );

  rclcpp::shutdown();

  return 0;
}