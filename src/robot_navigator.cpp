#include <nav_msgs/GridCells.h>

#include <nearest_frontier_planner/robot_navigator.h>
#include <nearest_frontier_planner/exploration_planner.h>

#include <set>
#include <map>

#define FREQUENCY 0.5

using namespace ros;
using namespace tf;

RobotNavigator::RobotNavigator() {
  NodeHandle robotNode;

  mStopServer = robotNode.advertiseService(NAV_STOP_SERVICE,
      &RobotNavigator::receiveStop, this);
  mPauseServer = robotNode.advertiseService(NAV_PAUSE_SERVICE,
      &RobotNavigator::receivePause, this);

  NodeHandle navigatorNode("~/");

  // Get parameters
  navigatorNode.param("exploration_strategy", mExplorationStrategy,
      std::string("NearestFrontierPlanner"));

  robotNode.param("map_frame", mMapFrame, std::string("map"));
  robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
  robotNode.param("explore_action_topic", mExploreActionTopic,
      std::string(NAV_EXPLORE_ACTION));

  // Apply tf_prefix to all used frame-id's
  mRobotFrame = mTfListener.resolve(mRobotFrame);
  mMapFrame = mTfListener.resolve(mMapFrame);

  try {
    mPlanLoader = new PlanLoader("nav2d_navigator", "ExplorationPlanner");
    mExplorationPlanner = mPlanLoader->createInstance(mExplorationStrategy);
    ROS_INFO("Successfully loaded exploration strategy [%s].",
        mExplorationStrategy.c_str());

    mExploreActionServer = new ExploreActionServer(mExploreActionTopic,
        boost::bind(&RobotNavigator::receiveExploreGoal, this, _1), false);
    mExploreActionServer->start();
  } catch ( pluginlib::PluginlibException& ex ) {
    ROS_ERROR("Failed to load exploration plugin! Error: %s", ex.what());
    mExploreActionServer = NULL;
    mPlanLoader = NULL;
  }

  mHasNewMap = false;
  mIsStopped = false;
  mIsPaused = false;
  goal_publisher_ = robotNode.advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 2);
  map_sub_ = robotNode.subscribe("/move_base_node/global_costmap/costmap", 1,
      &RobotNavigator::mapCallback, this);
}

RobotNavigator::~RobotNavigator() {
  delete mExploreActionServer;
  mExplorationPlanner.reset();
  delete mPlanLoader;
}

bool RobotNavigator::receiveStop(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  mIsStopped = true;
  res.success = true;
  res.message = "Navigator received stop signal.";
  return true;
}

bool RobotNavigator::receivePause(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  if ( mIsPaused ) {
    mIsPaused = false;
    res.success = false;
    res.message = "Navigator continues.";
  } else {
    mIsPaused = true;
    res.success = true;
    res.message = "Navigator pauses.";
  }
  return true;
}

bool RobotNavigator::preparePlan() {
  // Where am I?
  if ( !setCurrentPosition() ) return false;

  // Clear robot footprint in map

  return true;
}

void RobotNavigator::stop() {
  mIsPaused = false;
  mIsStopped = false;
}

void RobotNavigator::receiveExploreGoal(
    const nearest_frontier_planner::ExploreGoal::ConstPtr &goal) {
  // ROS_INFO(__func__);
  Rate loopRate(FREQUENCY);
  while ( true ) {
    // Check if we are asked to preempt
    if ( !ok() || mExploreActionServer->isPreemptRequested() || mIsStopped ) {
      ROS_INFO("Exploration has been preempted externally.");
      mExploreActionServer->setPreempted();
      stop();
      return;
    }

    // Where are we now
    if ( !setCurrentPosition() ) {
      ROS_ERROR("Exploration failed, could not get current position.");
      mExploreActionServer->setAborted();
      stop();
      return;
    }

    mGoalPoint = mCurrentMap.getSize();
    if ( preparePlan() ) {
      ROS_INFO("exploration: start = %u, end = %u.", mStartPoint, mGoalPoint);
      int result = mExplorationPlanner->findExplorationTarget(&mCurrentMap,
          mStartPoint, mGoalPoint);
      ROS_INFO("exploration: start = %u, end = %u.", mStartPoint, mGoalPoint);
      unsigned int x_index = 0, y_index = 0;
      mCurrentMap.getCoordinates(x_index, y_index, mStartPoint);
      ROS_INFO("start: x = %u, y = %u", x_index, y_index);
      unsigned int x_stop = 0, y_stop = 0;


      double x_ = x_index * mCurrentMap.getResolution() +
        mCurrentMap.getOriginX();
      double y_ = y_index * mCurrentMap.getResolution() +
        mCurrentMap.getOriginY();
      ROS_INFO("start: x = %f, y = %f", x_, y_);

      double x, y;
      if ( mGoalPoint == mCurrentMap.getSize() ) {
        x = (x_index+1) * mCurrentMap.getResolution() +
          mCurrentMap.getOriginX();
        y = y_index * mCurrentMap.getResolution() +
          mCurrentMap.getOriginY();
      } else {
        mCurrentMap.getCoordinates(x_stop, y_stop, mGoalPoint);
        x = (x_stop+1) * mCurrentMap.getResolution() +
          mCurrentMap.getOriginX();
        y = y_stop * mCurrentMap.getResolution() +
          mCurrentMap.getOriginY();
      }
      ROS_INFO("goal: x = %f, y = %f", x, y);

      geometry_msgs::PoseStamped goal_base;
      goal_base.header.stamp = ros::Time::now();
      goal_base.header.frame_id = "map";
      goal_base.pose.position.x = x;
      goal_base.pose.position.y = y;
      goal_base.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      goal_publisher_.publish(goal_base);
    }
    mHasNewMap = false;

    // Sleep remaining time
    spinOnce();
    loopRate.sleep();
    if ( loopRate.cycleTime() > ros::Duration(1.0 / FREQUENCY) )
      ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",
          FREQUENCY, loopRate.cycleTime().toSec());
  }
}

bool RobotNavigator::isLocalized() {
  return mTfListener.waitForTransform(mMapFrame,
      mRobotFrame, Time::now(), Duration(0.1));
}

bool RobotNavigator::setCurrentPosition() {
  // ROS_INFO(__func__);
  StampedTransform transform;
  try {
    mTfListener.lookupTransform(mMapFrame, mRobotFrame, Time(0), transform);
  } catch ( TransformException ex ) {
    ROS_ERROR("Could not get robot position: %s", ex.what());
    return false;
  }
  double world_x = transform.getOrigin().x();
  double world_y = transform.getOrigin().y();
  double world_theta = getYaw(transform.getRotation());

  unsigned int current_x = (world_x - mCurrentMap.getOriginX()) /
    mCurrentMap.getResolution();
  unsigned int current_y = (world_y - mCurrentMap.getOriginY()) /
    mCurrentMap.getResolution();
  unsigned int i;

  if ( !mCurrentMap.getIndex(current_x, current_y, i) ) {
    if ( mHasNewMap || !mCurrentMap.getIndex(current_x, current_y, i) ) {
      ROS_ERROR("Is the robot out of the map?");
      return false;
    }
  }
  mStartPoint = i;
  return true;
}

void RobotNavigator::mapCallback(const nav_msgs::OccupancyGrid& global_map) {
  if ( !mHasNewMap ) {
    // ROS_INFO(__func__);
    mCurrentMap.update(global_map);
    mCurrentMap.setLethalCost(50);
    mHasNewMap = true;
  }
}
