#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "stdint.h"

/**
 * Priority based arbiter, forwards highest priority commands
 */
class GoalSeek
{
public:
	// members
	
	// methods
  GoalSeek();
  
  void publish(double angular, double linear);

protected:
	
	// members
	ros::NodeHandle nh_, ph_;
	ros::Subscriber sub_0, sub_1, sub_2;
	ros::Publisher vel_pub_;
	
	int rate_; // update and publish rate (Hz)
	
	// methods
};

///////////////////////////////////////////////////////////////////////////

GoalSeek::GoalSeek(): ph_("~"), rate_(1), nh_()
{
	ph_.param("publish_rate", rate_, rate_);
	
	
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "GoalSeek constructor is called");
}


void GoalSeek::publish(double angular, double linear)
{
	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;
	vel_pub_.publish(vel);    
	return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GoalSeek");
  Arbiter arbiter;
  arbiter.arbitrate();
 
  return 0;
}
