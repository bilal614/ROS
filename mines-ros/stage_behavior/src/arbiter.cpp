#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "stdint.h"

/**
 * Priority based arbiter, forwards highest priority commands
 */
class Arbiter
{
public:
	// members
	
	// methods
  Arbiter();
  void teleopCallback(const geometry_msgs::Twist &msg);
  void escape_behaviorCallback(const geometry_msgs::Twist &msg);
  void cruise_behaviorCallback(const geometry_msgs::Twist &msg);
  
  void publish(double angular, double linear);
  void setPriority();
  void resetPriorityMaks();
  
  void arbitrate();

protected:
	
	enum Priority
    {
      TELE_OP,
      ESCAPE,
      CRUISE,
    };
    
	// members
	ros::NodeHandle nh_, ph_;
	ros::Subscriber sub_0, sub_1, sub_2;
	ros::Publisher vel_pub_;
	
	int rate_; // update and publish rate (Hz)
	int inputs_;
	
	geometry_msgs::Twist cmd_vel_tele_op, cmd_vel_escape, cmd_vel_cruise;
	Priority priority_;
	uint8_t priority_mask_;
	// methods
};

///////////////////////////////////////////////////////////////////////////

Arbiter::Arbiter(): ph_("~"), rate_(1), inputs_(3), nh_()
{
	ph_.param("publish_rate", rate_, rate_);
	ph_.param("inputs", inputs_, inputs_);
	
	priority_mask_ = 0x00;
	
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	sub_0 = nh_.subscribe("cmd_vel0", rate_, &Arbiter::teleopCallback, this);
	sub_1 = nh_.subscribe("cmd_vel1", rate_, &Arbiter::escape_behaviorCallback, this);
	sub_2 = nh_.subscribe("cmd_vel2", rate_, &Arbiter::cruise_behaviorCallback, this);
	
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "Arbiter constructor is called");
}

void Arbiter::resetPriorityMaks()
{
	priority_mask_ = 0x00;
	return; 
}

void Arbiter::publish(double angular, double linear)
{
	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;
	vel_pub_.publish(vel);    
	return;
}

void Arbiter::teleopCallback(const geometry_msgs::Twist &msg)
{
	priority_mask_ |= 0x01;
	cmd_vel_tele_op.angular.z = msg.angular.z;
	cmd_vel_tele_op.linear.x = msg.linear.x;
	return;
}

void Arbiter::escape_behaviorCallback(const geometry_msgs::Twist &msg)
{
	priority_mask_ |= 0x02;
	cmd_vel_escape.angular.z = msg.angular.z;
	cmd_vel_escape.linear.x = msg.linear.x;
	return;
}

void Arbiter::cruise_behaviorCallback(const geometry_msgs::Twist &msg)
{
	priority_mask_ |= 0x04;	
	cmd_vel_cruise.angular.z = msg.angular.z;
	cmd_vel_cruise.linear.x = msg.linear.x;
	return;
}

void Arbiter::setPriority()
{
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "priority_mask: " << (int)priority_mask_);
	if(priority_mask_ & 0x01)
	{
		priority_ = TELE_OP;
		publish(cmd_vel_tele_op.angular.z, cmd_vel_tele_op.linear.x);
		return;
	}
	else if(priority_mask_ & 0x02)
	{
		priority_ = ESCAPE;
		publish(cmd_vel_escape.angular.z, cmd_vel_escape.linear.x);
		return;
	}
	else if(priority_mask_ & 0x04)
	{
		priority_ = CRUISE;
		publish(cmd_vel_cruise.angular.z, cmd_vel_cruise.linear.x);
		return;
	}
	
}

void Arbiter::arbitrate()
{
	while(ros::ok())
	{
		ros::spinOnce();
		
		setPriority();
		resetPriorityMaks();
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbiter");
  Arbiter arbiter;
  arbiter.arbitrate();
 
  //ros::spin();

  return 0;
}
