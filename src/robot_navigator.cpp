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

  std::string serviceName;
  robotNode.param("map_service", serviceName, std::string("get_map"));
  mGetMapClient = robotNode.serviceClient<nav_msgs::GetMap>(serviceName);
  mStopServer = robotNode.advertiseService(NAV_STOP_SERVICE,
      &RobotNavigator::receiveStop, this);
  mPauseServer = robotNode.advertiseService(NAV_PAUSE_SERVICE,
      &RobotNavigator::receivePause, this);

  NodeHandle navigatorNode("~/");

  // Get parameters
  navigatorNode.param("map_inflation_radius", mInflationRadius, 1.0);
  navigatorNode.param("robot_radius", mRobotRadius, 0.3);
  navigatorNode.param("exploration_strategy", mExplorationStrategy,
      std::string("NearestFrontierPlanner"));
  mCostObstacle = 100;
  mCostLethal =
    (1.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;

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
  mStatus = NAV_ST_IDLE;
  mCellInflationRadius = 0;
  goal_publisher_ = robotNode.advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 2);
}

RobotNavigator::~RobotNavigator() {
  delete mExploreActionServer;
  mExplorationPlanner.reset();
  delete mPlanLoader;
}

bool RobotNavigator::getMap() {
  if ( mHasNewMap ) return true;

  if ( !mGetMapClient.isValid() ) {
    ROS_ERROR("GetMap-Client is invalid!");
    return false;
  }

  nav_msgs::GetMap srv;
  if ( !mGetMapClient.call(srv) ) {
    ROS_INFO("Could not get a map.");
    return false;
  }
  mCurrentMap.update(srv.response.map);

  if ( mCellInflationRadius == 0 ) {
    ROS_INFO("Navigator is now initialized.");
    mCellInflationRadius = mInflationRadius / mCurrentMap.getResolution();
    mCellRobotRadius = mRobotRadius / mCurrentMap.getResolution();
    mInflationTool.computeCaches(mCellInflationRadius);
    mCurrentMap.setLethalCost(mCostLethal);
  }

  mHasNewMap = true;
  return true;
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

typedef std::multimap<double, unsigned int> Queue;
typedef std::pair<double, unsigned int> Entry;

bool RobotNavigator::preparePlan() {
  // Get the current map
  // return false;
  if ( !getMap() ) {
    if ( mCellInflationRadius == 0 ) return false;
    ROS_WARN("Could not get a new map, trying to go with the old one...");
  }

  // Where am I?
  if ( !setCurrentPosition() ) return false;

  // Clear robot footprint in map
  unsigned int x = 0, y = 0;
  if ( mCurrentMap.getCoordinates(x, y, mStartPoint) ) {
    for ( int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++ )
      for ( int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++ )
        mCurrentMap.setData(x+i, y+j, 0);
  }

  mInflationTool.inflateMap(&mCurrentMap);
  return true;
}

void RobotNavigator::stop() {
  mStatus = NAV_ST_IDLE;
  mIsPaused = false;
  mIsStopped = false;
}

void RobotNavigator::receiveExploreGoal(
    const nearest_frontier_planner::ExploreGoal::ConstPtr &goal) {
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
    mHasNewMap = false;
    if ( !setCurrentPosition() ) {
      ROS_ERROR("Exploration failed, could not get current position.");
      mExploreActionServer->setAborted();
      stop();
      return;
    }

      if ( preparePlan() ) {
        int result = mExplorationPlanner->findExplorationTarget(&mCurrentMap,
            mStartPoint, mGoalPoint);
        ROS_INFO("exploration: start = %u, end = %u.", mStartPoint, mGoalPoint);
        unsigned int x_index = 0, y_index = 0;
        mCurrentMap.getCoordinates(x_index, y_index, mStartPoint);
        ROS_INFO("start: x = %u, y = %u", x_index, y_index);
        unsigned int x_stop = 0, y_stop = 0;

        mCurrentMap.getCoordinates(x_stop, y_stop, mGoalPoint);

        double x_ = x_index * mCurrentMap.getResolution() +
          mCurrentMap.getOriginX();
        double y_ = y_index * mCurrentMap.getResolution() +
          mCurrentMap.getOriginY();
        ROS_INFO("start: x = %f, y = %f", x_, y_);

        double x = x_stop * mCurrentMap.getResolution() +
          mCurrentMap.getOriginX();
        double y = y_stop * mCurrentMap.getResolution() +
          mCurrentMap.getOriginY();
        ROS_INFO("goal: x = %f, y = %f", x, y);

        geometry_msgs::PoseStamped goal_base;
        goal_base.header.stamp = ros::Time::now();
        goal_base.header.frame_id = "map";
        goal_base.pose.position.x = x;
        goal_base.pose.position.y = y;
        goal_base.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        goal_publisher_.publish(goal_base);
      }

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
    if ( mHasNewMap || !getMap() ||
        !mCurrentMap.getIndex(current_x, current_y, i) ) {
      ROS_ERROR("Is the robot out of the map?");
      return false;
    }
  }
  mStartPoint = i;
  return true;
}
