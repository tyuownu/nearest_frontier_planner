#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.h>
#include <nearest_frontier_planner/MoveToPosition2DAction.h>
#include <nearest_frontier_planner/ExploreAction.h>
#include <nearest_frontier_planner/GetFirstMapAction.h>
#include <nearest_frontier_planner/LocalizeAction.h>

#include <nearest_frontier_planner/grid_map.h>
#include <nearest_frontier_planner/commands.h>
#include <nearest_frontier_planner/map_inflation_tool.h>
#include <nearest_frontier_planner/exploration_planner.h>

#include <queue>

typedef actionlib::SimpleActionServer<nearest_frontier_planner::MoveToPosition2DAction> MoveActionServer;
typedef actionlib::SimpleActionServer<nearest_frontier_planner::ExploreAction> ExploreActionServer;
typedef actionlib::SimpleActionServer<nearest_frontier_planner::GetFirstMapAction> GetMapActionServer;
typedef actionlib::SimpleActionServer<nearest_frontier_planner::LocalizeAction> LocalizeActionServer;
typedef pluginlib::ClassLoader<ExplorationPlanner> PlanLoader;

class RobotNavigator
{
public:
	RobotNavigator();
	~RobotNavigator();

	bool receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	void receiveMoveGoal(const nearest_frontier_planner::MoveToPosition2DGoal::ConstPtr &goal);
	void receiveExploreGoal(const nearest_frontier_planner::ExploreGoal::ConstPtr &goal);
	void receiveGetMapGoal(const nearest_frontier_planner::GetFirstMapGoal::ConstPtr &goal);
	void receiveLocalizeGoal(const nearest_frontier_planner::LocalizeGoal::ConstPtr &goal);

private:
	bool isLocalized();
	bool setCurrentPosition();
	bool getMap();
	void stop();
	bool correctGoalPose();
	bool generateCommand();
	bool preparePlan();
	bool createPlan();
	void publishPlan();

	// Everything related to ROS
	tf::TransformListener mTfListener;
	ros::ServiceClient mGetMapClient;
	ros::Subscriber mGoalSubscriber;
	ros::Publisher mPlanPublisher;
	ros::Publisher mCommandPublisher;
	ros::Publisher mMarkerPublisher;
	ros::ServiceServer mStopServer;
	ros::ServiceServer mPauseServer;

	std::string mMapFrame;
	std::string mRobotFrame;
	std::string mMoveActionTopic;
	std::string mExploreActionTopic;
	std::string mGetMapActionTopic;
	std::string mLocalizeActionTopic;

	MoveActionServer* mMoveActionServer;
	ExploreActionServer* mExploreActionServer;
	GetMapActionServer* mGetMapActionServer;
	LocalizeActionServer* mLocalizeActionServer;

	PlanLoader* mPlanLoader;

	// Current status and goals
	bool mHasNewMap;
	bool mIsPaused;
	bool mIsStopped;
	int mStatus;
	int mRobotID;
	unsigned int mGoalPoint;
	unsigned int mStartPoint;
	double mCurrentDirection;
	double mCurrentPositionX;
	double mCurrentPositionY;

	// Everything related to the global map and plan
	MapInflationTool mInflationTool;
	std::string mExplorationStrategy;
	boost::shared_ptr<ExplorationPlanner> mExplorationPlanner;
	GridMap mCurrentMap;
	double* mCurrentPlan;

	double mInflationRadius;
	double mRobotRadius;
	unsigned int mCellInflationRadius;
	unsigned int mCellRobotRadius;

	signed char mCostObstacle;
	signed char mCostLethal;

	double mNavigationGoalDistance;
	double mNavigationGoalAngle;
	double mNavigationHomingDistance;
	double mExplorationGoalDistance;
	double mMinReplanningPeriod;
	double mMaxReplanningPeriod;
};
