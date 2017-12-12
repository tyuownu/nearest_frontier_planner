#include <ros/ros.h>

#include <nearest_frontier_planner/robot_navigator.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Navigator");
  ros::NodeHandle n;

  RobotNavigator robNav;

  ros::spin();
  return 0;
}
