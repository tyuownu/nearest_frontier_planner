#include <nav_msgs/GridCells.h>

#include <nearest_frontier_planner/robot_navigator.h>

#include <set>
#include <map>


RobotNavigator::RobotNavigator() {
  ros::NodeHandle robot_node;

  stop_server_ = robot_node.advertiseService(NAV_STOP_SERVICE,
      &RobotNavigator::receiveStop, this);
  pause_server_ = robot_node.advertiseService(NAV_PAUSE_SERVICE,
      &RobotNavigator::receivePause, this);

  ros::NodeHandle robot_node_pravite("~/");

  robot_node_pravite.param("map_frame", map_frame_, std::string("map"));
  robot_node_pravite.param("robot_frame", robot_frame_, std::string("robot"));
  robot_node_pravite.param("update_frequency", update_frequency_, 1.0);
  robot_node_pravite.param("explore_action_topic", explore_action_topic_,
      std::string(NAV_EXPLORE_ACTION));

  // Apply tf_prefix to all used frame-id's
  robot_frame_ = tf_listener_.resolve(robot_frame_);
  map_frame_ = tf_listener_.resolve(map_frame_);

  explore_action_server_ = new ExploreActionServer(explore_action_topic_,
      boost::bind(&RobotNavigator::receiveExploreGoal, this, _1), false);
  explore_action_server_->start();

  has_new_map_ = false;
  is_stopped_ = false;
  is_paused_ = false;
  goal_publisher_ = robot_node.advertise<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 2);
  map_sub_ = robot_node.subscribe("/move_base_node/global_costmap/costmap", 1,
      &RobotNavigator::mapCallback, this);
}

RobotNavigator::~RobotNavigator() {
  delete explore_action_server_;
}

bool RobotNavigator::receiveStop(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  is_stopped_ = true;
  res.success = true;
  res.message = "Navigator received stop signal.";
  return true;
}

bool RobotNavigator::receivePause(std_srvs::Trigger::Request &req,
    std_srvs::Trigger::Response &res) {
  if ( is_paused_ ) {
    is_paused_ = false;
    res.success = false;
    res.message = "Navigator continues.";
  } else {
    is_paused_ = true;
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
  is_paused_ = false;
  is_stopped_ = false;
}

void RobotNavigator::receiveExploreGoal(
    const nearest_frontier_planner::ExploreGoal::ConstPtr &goal) {
  ros::Rate loop_rate(update_frequency_);
  while ( true ) {
    // Check if we are asked to preempt
    if ( !ros::ok() || explore_action_server_->isPreemptRequested() || is_stopped_ ) {
      ROS_INFO("Exploration has been preempted externally.");
      explore_action_server_->setPreempted();
      stop();
      return;
    }

    // Where are we now
    if ( !setCurrentPosition() ) {
      ROS_ERROR("Exploration failed, could not get current position.");
      explore_action_server_->setAborted();
      stop();
      return;
    }

    goal_point_ = current_map_.getSize();
    if ( preparePlan() ) {
      ROS_DEBUG("exploration: start = %u, end = %u.", start_point_, goal_point_);
      int result = exploration_planner_.findExplorationTarget(&current_map_,
          start_point_, goal_point_);
      ROS_DEBUG("exploration: start = %u, end = %u.", start_point_, goal_point_);
      unsigned int x_start = 0, y_start = 0;
      current_map_.getCoordinates(x_start, y_start, start_point_);
      ROS_DEBUG("start: x = %u, y = %u", x_start, y_start);
      unsigned int x_stop = 0, y_stop = 0;


      double x_ = x_start * current_map_.getResolution() +
        current_map_.getOriginX();
      double y_ = y_start * current_map_.getResolution() +
        current_map_.getOriginY();
      ROS_DEBUG("start: x = %f, y = %f", x_, y_);

      double x, y;
      if ( goal_point_ == current_map_.getSize() ) {
        x = x_start * current_map_.getResolution() +
          current_map_.getOriginX();
        y = y_start * current_map_.getResolution() +
          current_map_.getOriginY();
      } else {
        current_map_.getCoordinates(x_stop, y_stop, goal_point_);
        x = x_stop * current_map_.getResolution() +
          current_map_.getOriginX();
        y = y_stop * current_map_.getResolution() +
          current_map_.getOriginY();
      }
      ROS_DEBUG("goal: x = %f, y = %f", x, y);

      geometry_msgs::PoseStamped goal_base;
      goal_base.header.stamp = ros::Time::now();
      goal_base.header.frame_id = "map";
      goal_base.pose.position.x = x;
      goal_base.pose.position.y = y;
      goal_base.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      goal_publisher_.publish(goal_base);
    }
    has_new_map_ = false;

    // Sleep remaining time
    ros::spinOnce();
    loop_rate.sleep();
    if ( loop_rate.cycleTime() > ros::Duration(1.0 / update_frequency_) )
      ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",
          update_frequency_, loop_rate.cycleTime().toSec());
  }
}

bool RobotNavigator::setCurrentPosition() {
  tf::StampedTransform transform;
  try {
    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
  } catch ( tf::TransformException ex ) {
    ROS_ERROR("Could not get robot position: %s", ex.what());
    return false;
  }
  double world_x = transform.getOrigin().x();
  double world_y = transform.getOrigin().y();
  double world_theta = getYaw(transform.getRotation());

  unsigned int current_x = (world_x - current_map_.getOriginX()) /
    current_map_.getResolution();
  unsigned int current_y = (world_y - current_map_.getOriginY()) /
    current_map_.getResolution();
  unsigned int i;

  if ( !current_map_.getIndex(current_x, current_y, i) ) {
    if ( has_new_map_ || !current_map_.getIndex(current_x, current_y, i) ) {
      ROS_ERROR("Is the robot out of the map?");
      return false;
    }
  }
  start_point_ = i;
  return true;
}

void RobotNavigator::mapCallback(const nav_msgs::OccupancyGrid& global_map) {
  if ( !has_new_map_ ) {
    current_map_.update(global_map);
    current_map_.setLethalCost(80);
    has_new_map_ = true;
  }
}
