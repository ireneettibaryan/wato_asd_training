#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {

  public:

    PlannerNode();

  private:

    robot::PlannerCore planner_;

    // Represents the two states of the planner
    enum class State {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };

    State state_;


    // Structure used in the A* priority queue
    struct GridNode {

      int index;
      double f_score;

      GridNode(int i, double f)
      : index(i), f_score(f)
      {
      }
    };


    // Makes the priority queue choose the node
    // with the smallest f-score first
    struct CompareNode {

      bool operator()(
        const GridNode & a,
        const GridNode & b) const
      {
        return a.f_score > b.f_score;
      }
    };


    // Receives the global occupancy map
    rclcpp::Subscription<
      nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;


    // Receives the requested goal position
    rclcpp::Subscription<
      geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;


    // Receives the current robot position
    rclcpp::Subscription<
      nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


    // Publishes the planned path
    rclcpp::Publisher<
      nav_msgs::msg::Path>::SharedPtr path_pub_;


    // Periodically checks the planner state
    rclcpp::TimerBase::SharedPtr timer_;


    // Stores the most recently received information
    nav_msgs::msg::OccupancyGrid current_map_;

    geometry_msgs::msg::PointStamped goal_;

    geometry_msgs::msg::Pose robot_pose_;


    bool have_map_;
    bool have_odom_;
    bool goal_received_;


    // Callback functions
    void mapCallback(
      const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void goalCallback(
      const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void odomCallback(
      const nav_msgs::msg::Odometry::SharedPtr msg);

    void timerCallback();


    // Planner functions
    void planPath();

    bool goalReached() const;


    // Converts world coordinates into map-grid coordinates
    bool worldToGrid(
      double world_x,
      double world_y,
      int & grid_x,
      int & grid_y) const;


    // Converts map-grid coordinates back into world coordinates
    void gridToWorld(
      int grid_x,
      int grid_y,
      double & world_x,
      double & world_y) const;


    // Checks whether A* is allowed to enter a grid cell
    bool isTraversable(
      int grid_x,
      int grid_y) const;


    // Calculates the A* heuristic
    double heuristic(
      int x1,
      int y1,
      int x2,
      int y2) const;
};

#endif