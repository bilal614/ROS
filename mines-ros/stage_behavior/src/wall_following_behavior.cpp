#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include "stage_behavior/State.h"

#define PI 3.141592

enum State
{
  IDLE,
  FOLLOW_WALL,
};

/**
 * Wall following behavior makes the robot follow along obstacle
 */
class WallFollowBehavior
{
public:
	// methods
	WallFollowBehavior();
	void publish();
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
	void stateCallback(const stage_behavior::State& state_msg);
	bool isTriggered(void);
	void update(void);

protected:
	// members
	ros::NodeHandle nh_,ph_;
    ros::Subscriber scan_sub_, state_sub_;
    ros::Publisher vel_pub_;
    ros::Timer timer_;
    State state_; // state of escape procedure
	ros::Time until_; // time until next state
	double wallDistance; // Desired distance from the wall.
	double e;            // Difference between desired distance from the wall and actual distance.
	double diffE;     // Derivative element for PD controller;
	double maxSpeed;     // Maximum speed of robot.
	double P;            // k_P Constant for PD controller.
	double D;            // k_D Constant for PD controller.
	double angleCoef;    // Coefficient for P controller.
	int direction;      // 1 for wall on the right side of the robot (-1 for the left one).
	double angleMin;     // Angle, at which was measured the shortest distance.
	double distFront;    // Distance, measured by ranger in front of robot.
	int rate_; // update and publish rate (Hz)
	bool intermediate_goal_reached; // becomes true when intermediate goal is reached and is reset once it reaches intermediate goal
	bool scan_received_; // at least one scan has been received
	double closest_distance_; // distance to closest object (m)
	double robot_size_; // size or diameter of robot (m)
	double bump_distance_; // robot size plus safety margin (m)
};

///////////////////////////////////////////////////////////////////////////

WallFollowBehavior::WallFollowBehavior():
  ph_("~"),
  rate_(200),
  state_(IDLE),
  scan_received_(false),
  robot_size_(0.33),
  bump_distance_(0.5),
  wallDistance(0.5),
  maxSpeed(0.2),
  direction(-1),
  P(10.0),
  D(5.0),
  angleCoef(1.0),
  e(0.0),
  angleMin(0.0), 
  nh_()//angle, at which was measured the shortest distance
{
  ph_.param("publish_rate", rate_, rate_);
  ph_.param("bump_distance", bump_distance_, bump_distance_);
  ph_.param("robot_size", robot_size_, robot_size_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_1", rate_);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &WallFollowBehavior::scanCallback, this);
  state_sub_ = nh_.subscribe("/state", 1000, &WallFollowBehavior::stateCallback, this);
  
  timer_ = nh_.createTimer(ros::Duration(1.0/rate_), boost::bind(&WallFollowBehavior::update, this));
}

void WallFollowBehavior::stateCallback(const stage_behavior::State& state_msg)
{
	if(state_msg.state == true)
	{
		state_ = FOLLOW_WALL;
	}
	else
	{
		state_ = IDLE;
	}
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "state: " << (int)state_);
	return;
}

void WallFollowBehavior::scanCallback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(state_ == IDLE)
	{
		closest_distance_ = msg->range_max; 
		for (int i = 0; i < (int) msg->ranges.size(); i++) 
		{
			double range = msg->ranges[i];
			if ((range > msg->range_min) && (range > robot_size_) && (range < closest_distance_)) 
			{ // closest distance to obstacle, excluding parts of the robot
				closest_distance_ = range;
			}
		}
		scan_received_ = true; // we have received a scan
		if(isTriggered()){state_ = FOLLOW_WALL;}
		return;
	}
	
	if(state_ == FOLLOW_WALL)
	{
		int size = msg->ranges.size();

		//Variables whith index of highest and lowest value in array.
		//Walls may occur either side of the robot, we want to allow for that.
		//Thus, we use the entire range of the laser scan.
		//int minIndex = size*(direction+1)/4;
		//int maxIndex = size*(direction+3)/4;
		int minIndex = 0;
		int maxIndex = size;
		
		bool keepStateToFollowWall = true;
		//This cycle goes through array and finds minimum
		for(int i = minIndex; i < maxIndex; i++)
		{
			//check if we still are following wall
			
			if ((msg->ranges[i] < msg->ranges[minIndex]) && (msg->ranges[i] > 0.0))
			{
				minIndex = i;
			}
		}
		for(int i = 0; i < size; i++){
			if (msg->ranges[i] != msg->ranges[minIndex]) {
				keepStateToFollowWall = false;
			}
		}
		//Calculation of angles from indexes and storing data to class variables.
		angleMin = (minIndex-size/2)*msg->angle_increment;
		double distMin;
		distMin = msg->ranges[minIndex];
		distFront = msg->ranges[size/2];
		diffE = (distMin - wallDistance) - e;
		e = distMin - wallDistance;
		scan_received_ = true; // we have received a scan
		
		return;
	}
}


void WallFollowBehavior::publish()
{
	geometry_msgs::Twist msg;

	msg.angular.z = direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2);    //PD controller

	if (distFront < wallDistance){
		msg.linear.x = 0;
	}
	else if (distFront < wallDistance * 2){
		msg.linear.x = 0.5*maxSpeed;
	}
	else if (fabs(angleMin)>1.75){
		msg.linear.x = 0.4*maxSpeed;
	} 
	else {
		msg.linear.x = maxSpeed;
	}
	vel_pub_.publish(msg);
}

bool WallFollowBehavior::isTriggered(void) {
  // check if behavior is triggered
  return scan_received_ && (closest_distance_ < bump_distance_);
}

void WallFollowBehavior::update(void)
{
  geometry_msgs::Twist twist_msg;
  ros::Time now = ros::Time::now();
  switch (state_)
  {
	case IDLE:      
		break;
    case FOLLOW_WALL:
		publish();
		break;
    default:
		break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wall_follow_behavior");
  WallFollowBehavior wall_follow_behavior;
  ros::spin();

  return 0;
}
