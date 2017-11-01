#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include "steering.hpp"
#include <math.h>
#include <iostream>
#include <exception>


const double lookaheadRadius = sqrt(3.0);
struct point {double x; double y;};
template<typename T> struct pair{ T p1; T p2; };

double getSlope_L(double x1, double x2, double y1, double y2);//returns the slope of a line, constructed from 2 given points
double getIntercept_L(double x, double y, double slope);//returns the y-intercept value of a line, given a point and slope
pair<point> solveCircleLineQuad(double x1, double x2, double y1, double y2);//Solve for intersection points of the line L defined by 
//2 points and the circle C with radius defined by lookaheadRadius
double computeAngleE(point point1, point point2, double currentTheta);

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
		//listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
		//pose_listener.lookupTransform("/map", "/base_link", ros::Time(0), position_transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
		
	/*
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "\nMap Message: x-position: " << transform.getOrigin().x()
		<<", y-position:" << transform.getOrigin().y());
	*/
	
	//x and y represent current position of the robot, th represents orientation of the robot	
	double x = position_transform.getOrigin().x();
	double y = position_transform.getOrigin().y();
	double th = tf::getYaw(position_transform.getRotation());
	
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "\nCurrent Position: x-position: " << x
		<<", y-position:" << y
		<< ", rotation: " << th);
	
	//steering.setGoal(transform.getOrigin().x(), transform.getOrigin().y(), M_PI);
    geometry_msgs::Twist vel_msg;
    
    float deltaX = transform.getOrigin().x() - position_transform.getOrigin().x();
    float deltaY = transform.getOrigin().y() - position_transform.getOrigin().y();
    
    float rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
        
    vel_msg.linear.x = 0.1;
    //vel_msg.angular.z = 0.1;
    stage_vel.publish(vel_msg);
    rate.sleep();
	
	//testing
	pair<point> Result = solveCircleLineQuad(4, -4, -1, 4);
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "\nPoint1: x = " << Result.p1.x 
		<< ", y = " << Result.p1.y
		<< "\nPoint2: x = " << Result.p2.x 
		<< ", y = " << Result.p2.y);
		
	point robotPoint;
	robotPoint.x = x;
	robotPoint.y = y;
	double angleE = computeAngleE(Result.p1, robotPoint, th);
	
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "\nAngle e: " << angleE*180/M_PI);
	
  }
  return 0;
};

double getSlope_L(double x1, double x2, double y1, double y2)
{
	return (y2 - y1)/(x2 - x1);
}

double getIntercept_L(double x, double y, double slope)
{
	return y - slope*x;
}

pair<point> solveCircleLineQuad(double x1, double x2, double y1, double y2)
{

	pair<point> result;
	
	double m = getSlope_L(x1, x2, y1, y2);
	double B = getIntercept_L(x1, y1, m);
	double a = 1 + pow(m, 2);
	double b = 2*B*m;
	double c = pow(B, 2) - pow(lookaheadRadius, 2);
	
	if(a<0.000001)    // ==0
	{
		if(b>0.000001)  // !=0
		{
			result.p1.x=result.p2.x=-c/b;
		}
		else
			if(c>0.00001) throw "no solutions";
		return result;
	}

	double delta=b*b-4*a*c;
	if(delta>=0)
	{
		result.p1.x=(-b-sqrt(delta))/2/a;
		result.p2.x=(-b+sqrt(delta))/2/a;
		result.p1.y = m*result.p1.x + B;
		result.p2.y = m*result.p2.x + B;
	}
	else
	{
		throw "non-real solutions";
	}

	return result;
}

double computeAngleE(point point1, point robot_point, double currentTheta)
{
	return (atan2((point1.y - robot_point.y), (point1.x - robot_point.x)) - currentTheta);
}
