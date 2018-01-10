#ifndef ROBOT_NAVIGATOR_H_
#define ROBOT_NAVIGATOR_H_
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_loader.h>
#include <nearest_frontier_planner/ExploreAction.h>

#include <nearest_frontier_planner/grid_map.h>
#include <nearest_frontier_planner/commands.h>
#include <nearest_frontier_planner/nearest_frontier_planner.h>

#include <move_base_msgs/MoveBaseAction.h>


typedef actionlib::SimpleActionServer<nearest_frontier_planner::ExploreAction> ExploreActionServer;

class RobotNavigator
{
 public:
  RobotNavigator();
  ~RobotNavigator();

  bool receiveStop(std_srvs::Trigger::Request &req,
      std_srvs::Trigger::Response &res);
  bool receivePause(std_srvs::Trigger::Request &req,
      std_srvs::Trigger::Response &res);
  void receiveExploreGoal(
      const nearest_frontier_planner::ExploreGoal::ConstPtr &goal);
  void mapCallback(const nav_msgs::OccupancyGrid& global_map);
  void scanCallback(const sensor_msgs::LaserScan& scan);
  int scoreLine(double, double);

 private:
  bool setCurrentPosition();
  void stop();
  bool preparePlan();

  // Everything related to ROS
  tf::TransformListener tf_listener_;
  ros::ServiceServer stop_server_;
  ros::ServiceServer pause_server_;

  std::string map_frame_;
  std::string robot_frame_;
  std::string explore_action_topic_;

  ExploreActionServer* explore_action_server_;

  // Current status and goals
  bool has_new_map_;
  bool is_paused_;
  bool is_stopped_;
  unsigned int goal_point_;
  unsigned int start_point_;
  double update_frequency_;

  double longest_distance_;
  double angles_;

  // Everything related to the global map and plan
  NearestFrontierPlanner exploration_planner_;
  GridMap current_map_;

  ros::Publisher goal_publisher_;
  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
};
#endif  // end ROBOT_NAVIGATOR_H_
