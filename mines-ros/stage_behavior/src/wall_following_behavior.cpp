#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

/**
 * Escape behavior makes the robot backoff and turn when too close to obstacle
 */
class WallFollowBehavior
{
public:
	// members

	// methods
  WallFollowBehavior();

protected:
	// members
	ros::NodeHandle nh_,ph_;
    ros::Subscriber scan_sub_;
    ros::Publisher vel_pub_;
	
	int rate_; // update and publish rate (Hz)

    enum State
    {
      IDLE,
      FOLLOW_WALL,
    };
    State state_; // state of escape procedure
	bool intermediate_goal_reached; // becomes true when intermediate goal is reached and is reset once it reaches intermediate goal

	bool scan_received_; // at least one scan has been received
	double closest_distance_; // distance to closest object (m)
	double robot_size_; // size or diameter of robot (m)
	double bump_distance_; // robot size plus safety margin (m)
    
	
	// methods
	void publish(double angular, double linear);
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
	bool isTriggered(void);
	void update(void);
};

///////////////////////////////////////////////////////////////////////////

WallFollowBehavior::WallFollowBehavior():
  ph_("~"),
  rate_(50),
  state_(IDLE),
  scan_received_(false),
  robot_size_(0.33),
  bump_distance_(1.0),
{
  ph_.param("publish_rate", rate_, rate_);
  ph_.param("bump_distance", bump_distance_, bump_distance_);
  ph_.param("robot_size", robot_size_, robot_size_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &EscapeBehavior::scanCallback, this);
  timer_ = nh_.createTimer(ros::Duration(1.0/rate_), boost::bind(&EscapeBehavior::update, this));
}
