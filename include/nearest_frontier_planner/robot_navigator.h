#ifndef ROBOT_NAVIGATOR_H_
#define ROBOT_NAVIGATOR_H_
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_loader.h>
#include <nearest_frontier_planner/ExploreAction.h>

#include <nearest_frontier_planner/grid_map.h>
#include <nearest_frontier_planner/commands.h>
#include <nearest_frontier_planner/exploration_planner.h>

#include <move_base_msgs/MoveBaseAction.h>


typedef actionlib::SimpleActionServer<nearest_frontier_planner::ExploreAction> ExploreActionServer;
typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;

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

 private:
  bool isLocalized();
  bool setCurrentPosition();
  void stop();
  bool preparePlan();

  // Everything related to ROS
  tf::TransformListener mTfListener;
  ros::ServiceServer mStopServer;
  ros::ServiceServer mPauseServer;

  std::string mMapFrame;
  std::string mRobotFrame;
  std::string mExploreActionTopic;

  ExploreActionServer* mExploreActionServer;

  PlanLoader* mPlanLoader;

  // Current status and goals
  bool mHasNewMap;
  bool mIsPaused;
  bool mIsStopped;
  unsigned int mGoalPoint;
  unsigned int mStartPoint;

  // Everything related to the global map and plan
  std::string mExplorationStrategy;
  boost::shared_ptr<ExplorationPlanner> mExplorationPlanner;
  GridMap mCurrentMap;

  ros::Publisher goal_publisher_;
  ros::Subscriber map_sub_;
};
#endif  // end ROBOT_NAVIGATOR_H_
