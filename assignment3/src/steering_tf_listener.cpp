#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include "steering.hpp"

int main(int argc, char** argv){
  ros::init(argc, argv, "local_planner");

  ros::NodeHandle nh;

  Steering steering(nh);

  tf::TransformListener listener;

  ros::Rate rate(1000.0);
  while (nh.ok()){
    tf::StampedTransform transform;
    try{
		listener.lookupTransform("/map", "/goal",
          ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

	steering.setGoal(transform.getOrigin().x(), transform.getOrigin().y(), M_PI);
    rate.sleep();
  }
  return 0;
};

