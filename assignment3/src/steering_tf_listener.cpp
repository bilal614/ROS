#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include "steering.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "local_planner");

  ros::NodeHandle nh;

  //Steering steering(nh);
	ros::Publisher stage_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	
  tf::TransformListener listener;//Listening to Goal related transformation between parent map and child odom
  tf::TransformListener pose_listener;//For current position

  ros::Rate rate(1000.0);
  while (nh.ok()){
    tf::StampedTransform transform;
    tf::StampedTransform position_transform;
    try{
		listener.lookupTransform("/map", "/odom",
          ros::Time(0), transform);
		pose_listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), position_transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
		
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "\nGoal Message: x-position: " << transform.getOrigin().x()
		<<", y-position:" << transform.getOrigin().y());
	
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "\nCurrent Position: x-position: " << position_transform.getOrigin().x()
		<<", y-position:" << position_transform.getOrigin().y());
	
	//steering.setGoal(transform.getOrigin().x(), transform.getOrigin().y(), M_PI);
    geometry_msgs::Twist vel_msg;
    
    float deltaX = transform.getOrigin().x() - position_transform.getOrigin().x();
    float deltaY = transform.getOrigin().y() - position_transform.getOrigin().y();
    
    float rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
        
    //vel_msg.angular.z = atan2(deltaY, deltaX);
    vel_msg.angular.z = 1.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
                                    
    vel_msg.linear.x = 0.75*rho;
    stage_vel.publish(vel_msg);
    rate.sleep();
    
   
	if(rho <= 0.2)
	{
		vel_msg.angular.z = 0;
		vel_msg.linear.x = 0;
		stage_vel.publish(vel_msg);
		return 0;
	}
	
  }
  return 0;
};

