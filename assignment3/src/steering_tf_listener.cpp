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
	
  tf::TransformListener listener;

  ros::Rate rate(1000.0);
  while (nh.ok()){
    tf::StampedTransform transform;
    try{
		listener.lookupTransform("/map", "/odom",
          ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
		
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "\nGoal Message: x-position: " << transform.getOrigin().x()
		<<", y-position:" << transform.getOrigin().y());
		
	//steering.setGoal(transform.getOrigin().x(), transform.getOrigin().y(), M_PI);
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    stage_vel.publish(vel_msg);
    rate.sleep();
  }
  return 0;
};

