#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include "stage_behavior/State.h"
#include "stdint.h"
#include <exception>
#include <math.h>

#define EPSILON 0.001
/**
 * Priority based arbiter, forwards highest priority commands
 */
class GoalSeek
{
public:
	// members
	
	// methods
  GoalSeek();
  void ListenTF();
  void publish(double angular, double linear);
  void goalSetCallback(const geometry_msgs::PoseStamped &msg);
  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  void wallCallback(const geometry_msgs::Twist &msg);
  geometry_msgs::Twist resolveVelocity();
  void sendFollowWallMessage();
  bool checkGoalReached();
  void goalSeekRun();
protected:
	
	// members
	ros::NodeHandle nh_, ph_;
	ros::Subscriber sub_, scan_sub_, wall_sub_;
	ros::Publisher vel_pub_, state_pub_;
	tf::TransformListener pose_listener;//For getting current position
	geometry_msgs::PoseStamped goal_;
	geometry_msgs::PointStamped origin_point;
	geometry_msgs::PointStamped current_point;
	tf::StampedTransform position_transform;
	geometry_msgs::Twist wall_vel_;
	
	int rate_; // update and publish rate (Hz)
	bool goal_received_;
	double bump_distance_; // robot size plus safety margin (m)
	
	bool on_track_;
	bool wall_hit_;
	bool goal_reached_;
	// methods
};

///////////////////////////////////////////////////////////////////////////

GoalSeek::GoalSeek(): ph_("~"), rate_(1), nh_(), 
pose_listener(), origin_point(), current_point(), goal_received_(false),
bump_distance_(1.0), on_track_(true), wall_hit_(false), goal_reached_(false), position_transform()
{
	ph_.param("publish_rate", rate_, rate_);
	
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_0", rate_);
	state_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/state", rate_);
	
	origin_point.header.frame_id = "odom";
	//we'll just use the most recent transform available for our simple example
	origin_point.header.stamp = ros::Time();
	origin_point.point.x = 0.0;
	origin_point.point.y = 0.0;
	origin_point.point.z = 0.0;
	
	scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 100, &GoalSeek::scanCallback, this);
	sub_ = nh_.subscribe("/move_base_simple/goal", rate_, &GoalSeek::goalSetCallback, this);
	wall_sub_ = nh_.subscribe("cmd_vel1", rate_, &GoalSeek::wallCallback, this);
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "GoalSeek constructor is called");
}

void GoalSeek::goalSetCallback(const geometry_msgs::PoseStamped &msg)
{
	if(!goal_received_)
	{
		goal_.pose.position.x = msg.pose.position.x;
		goal_.pose.position.y = msg.pose.position.y;
		goal_.pose.orientation.w = msg.pose.orientation.w;
		goal_received_ = true;
		
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed 
			<< "goal: x:" << goal_.pose.position.x << ", y: " << goal_.pose.position.y
			<< ", w: " << goal_.pose.orientation.w);
	}
}

void GoalSeek::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	if(checkGoalReached())
	{
		goal_reached_ = true;
		return;
	}
	//Determine if there is a wall in front of you
	int size = scan_msg->ranges.size();
	for(int i = (size/2)-10; i < (size/2)+10; i++)
	{
		if ((scan_msg->ranges[i] < bump_distance_) && (scan_msg->ranges[i] > 0.0))
		{
			stage_behavior::State state_;
			state_.state = true;
			on_track_ = false;
			wall_hit_ = true;
			state_pub_.publish(state_);
			return;
		}
	}
}

void GoalSeek::wallCallback(const geometry_msgs::Twist &msg)
{
	wall_vel_.angular.z =  msg.angular.z;
	wall_vel_.linear.x = msg.linear.x;
	return;
}

geometry_msgs::Twist GoalSeek::resolveVelocity()
{
	geometry_msgs::Twist velocity_msg;
	
	double goalX = goal_.pose.position.x;
	double goalY = goal_.pose.position.y;
	double currentX = -1*current_point.point.x;
	double currentY = -1*current_point.point.y;
	float deltaX = goal_.pose.position.x - position_transform.getOrigin().x();
	float deltaY = goal_.pose.position.y - position_transform.getOrigin().y();
	
	double angle = (atan2(deltaY, deltaX) - tf::getYaw(position_transform.getRotation()));
	
	velocity_msg.linear.x = 0.5;
	
	velocity_msg.angular.z = angle;
	/*For vertical line*/
	if(std::abs(goalY - currentY) < EPSILON)
	{
		velocity_msg.angular.z = 0;
	}
	
	return velocity_msg;
}

bool GoalSeek::checkGoalReached()
{
	double goalX = goal_.pose.position.x;
	double goalY = goal_.pose.position.y;
	double currentX = -1*current_point.point.x;
	double currentY = -1*current_point.point.y;
	if(std::abs(goalY - currentY) < EPSILON && std::abs(goalX - currentX) < EPSILON)
	{
		return true;	
	}
	else
	{
		return false;
	}
}
/*This function must be called in a repeating block eg. while(nh_.ok()) */
void GoalSeek::ListenTF()
{
	try{
		ros::Time now = ros::Time(0);
		pose_listener.waitForTransform("odom", "base_footprint",
		  now, ros::Duration(0.5));
		pose_listener.lookupTransform("odom", "base_footprint", now, position_transform);
		origin_point.header.stamp = ros::Time::now();
		pose_listener.transformPoint("base_footprint", origin_point, current_point);
	}
	catch (tf::TransformException &ex) {
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	double x = -1*current_point.point.x;
	double y = -1*current_point.point.y;
	double th = tf::getYaw(position_transform.getRotation());
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "x: " << x << ", y: " << y << ", th: " << th);
		
	geometry_msgs::Twist velocity = resolveVelocity();
	
	publish(velocity.angular.z, velocity.linear.x);	
}


void GoalSeek::publish(double angular, double linear)
{
	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;
	vel_pub_.publish(vel);    
	return;
}

void GoalSeek::sendFollowWallMessage()
{
	
}

void GoalSeek::goalSeekRun()
{
	while (nh_.ok())
	{
		if(goal_reached_)
		{
			break;
		}
		
		if(on_track_ && !wall_hit_ && !goal_reached_)
		{
			ListenTF();
		}
		else
		{
			publish(wall_vel_.angular.z, wall_vel_.linear.x);
		}	
		ros::spinOnce();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GoalSeek");
  GoalSeek goal_seek;
  goal_seek.goalSeekRun();
  
  return 0;
}
