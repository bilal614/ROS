#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "stdint.h"
#include <exception>

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

protected:
	
	// members
	ros::NodeHandle nh_, ph_;
	ros::Subscriber sub_;
	ros::Publisher vel_pub_;
	tf::TransformListener pose_listener;//For getting current position
	geometry_msgs::PoseStamped goal_;
	geometry_msgs::PointStamped origin_point;
	geometry_msgs::PointStamped current_point;
	
	int rate_; // update and publish rate (Hz)
	bool goal_received_;
	// methods
};

///////////////////////////////////////////////////////////////////////////

GoalSeek::GoalSeek(): ph_("~"), rate_(1), nh_(), 
pose_listener(), origin_point(), current_point(), goal_received_(false)
{
	ph_.param("publish_rate", rate_, rate_);
	
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	origin_point.header.frame_id = "odom";
	//we'll just use the most recent transform available for our simple example
	origin_point.header.stamp = ros::Time();
	origin_point.point.x = 0.0;
	origin_point.point.y = 0.0;
	origin_point.point.z = 0.0;
	
	sub_ = nh_.subscribe("/move_base_simple/goal", rate_, &GoalSeek::goalSetCallback, this);
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

/*This function must be called in a repeating block eg. while(nh_.ok()) */
void GoalSeek::ListenTF()
{
	while (nh_.ok())
	{
		try{
			//listener.lookupTransform("map", "odom", ros::Time(0.001), transform);
			//pose_listener.lookupTransform("odom", "base_footprint", ros::Time(0.001), position_transform);
			ros::Time now = ros::Time::now();
			origin_point.header.stamp = now;
			pose_listener.waitForTransform("odom", "base_footprint",
                              now, ros::Duration(1.0));
			pose_listener.transformPoint("base_footprint", origin_point, current_point);
		}
		catch (tf::TransformException &ex) {
		  ROS_ERROR("%s",ex.what());
		  ros::Duration(1.0).sleep();
		  continue;
		}
		
		//double x = position_transform.getOrigin().x();
		//double y = position_transform.getOrigin().y();
		double x = -1*current_point.point.x;
		double y = -1*current_point.point.y;
		//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			//<< "x: " << x << ", y: " << y);
		ros::spinOnce();
	}
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
  GoalSeek goal_seek;
  goal_seek.ListenTF();
  //ros::spin();
  
  return 0;
}
