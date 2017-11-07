#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include "steering.hpp"
#include <math.h>
#include <iostream>
#include <exception>
#include <nav_msgs/Path.h>
#include <vector>

/**Global Variables**/
const double lookaheadRadius = sqrt(64.0);
nav_msgs::Path pathMsg;
struct point {double x; double y;};
template<typename T> struct pair{ T p1; T p2; };
const int tri_sqr = 3;
int WayPoints = 3;
//geometry_msgs::PoseStamped poses[(tri_sqr+1)];
//point cart_points[(tri_sqr+1)];
tf::StampedTransform transform;
tf::StampedTransform position_transform;
int coordinateCounter = -1;
bool rec_msg = false;
std::vector<geometry_msgs::PoseStamped> poses;
std::vector<point> cart_points;

/**Functions**/
double getSlope_L(double x1, double x2, double y1, double y2);//returns the slope of a line, constructed from 2 given points
double getIntercept_L(double x, double y, double slope);//returns the y-intercept value of a line, given a point and slope
pair<point> solveCircleLineQuad(double x1, double x2, double y1, double y2);//Solve for intersection points of the line L defined by 
//2 points and the circle C with radius defined by lookaheadRadius
double computeAngleE(point point1, point point2, double currentTheta);
void PathMessageReceived(const nav_msgs::Path& msg);
point findCloserPoint(pair<point> pair_of_pts, point inspect);
//int followWaypoints(ros::Publisher stage_vel, geometry_msgs::Twist vel_msg,  int nr_of_waypoints);



int main(int argc, char** argv){
	ros::init(argc, argv, "local_planner");

	ros::NodeHandle nh;	
	/*
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\nPath Message: " << pathMsg);
	*/
	ros::Publisher stage_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	
	ros::Subscriber path_listener = nh.subscribe("/plan", 10, &PathMessageReceived);
	
	tf::TransformListener listener;
	tf::TransformListener pose_listener;//For getting current position
	geometry_msgs::Twist vel_msg;
	
	/*******************************/
		////////////////////////////
		
		///////////////////////////	
			poses.resize(tri_sqr+1);
			cart_points.resize(tri_sqr+1);
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\ncart_points: size: " << cart_points.size());
			//fill out waypoints 
			for (int i = 0; i < cart_points.size(); i++)
			{
				cart_points[i].x = 2*(i+1);
				poses[i].pose.position.x = 2*(i+1);//(2,4,6)
				//cart_points[i].x = (i%2)*3;
				//poses[i].pose.position.x = (i%2)*3;//(0,3,0,3)
				
				if(i == 1)
				//if(i > 1 && i < 4)
				{
					poses[i].pose.position.y = -6;
					cart_points[i].y = -6;
					//poses[i].pose.position.y = 4;
					//cart_points[i].y = 4;
				}
				else
				{
					poses[i].pose.position.y = -2;//(-2,-6,-2)
					cart_points[i].y = -2;
					//poses[i].pose.position.y = 0;//(0,0,4,4)
					//cart_points[i].y = 0; 
				}//(2,-2) (4,-6) (6,-2)
				//if(i == 3)
				if(i == 4)
				{
					
					cart_points[i].x = 2;
					poses[i].pose.position.x = 2;
					cart_points[i].y = -2;
					poses[i].pose.position.y = -2;
					
					/*
					cart_points[i].x = 0;
					poses[i].pose.position.x = 0;
					cart_points[i].y = 0;
					poses[i].pose.position.y = 0;
					*/
				}
				ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\ncart_points: i: " << i
				<<", x: " << cart_points[i].x
				<< ", y: " << cart_points[i].y);		
			}
			
			/*******************************/
	
	ros::Rate rate(1000.0);
	while (nh.ok())
	{
		
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
		
		geometry_msgs::Twist vel_msg;
		nav_msgs::Path Msg = *(ros::topic::waitForMessage<nav_msgs::Path>("/plan", nh));
		
		if(!rec_msg)
		{
			//We want to receive the message only once
			ros::spinOnce();
		}
		rate.sleep();
		/////////////////////////////////////////////
		
		//x and y represent current position of the robot, th represents orientation of the robot	
		double x = position_transform.getOrigin().x();
		double y = position_transform.getOrigin().y();
		double th = tf::getYaw(position_transform.getRotation());
		
		point robotPoint;
		robotPoint.x = x;
		robotPoint.y = y;
		float rho;
		pair<point> Result;	
		point closesPt;
		double angleE;
		vel_msg.linear.x = 0.25;
		if(coordinateCounter == -1)
		{
			Result = solveCircleLineQuad(robotPoint.x, 
				cart_points[coordinateCounter+1].x, robotPoint.y, 
				cart_points[coordinateCounter+1].y);
			/*
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\ncart_point[0]: x: " << cart_points[coordinateCounter+1].x 
			<< ", y:" << cart_points[coordinateCounter+1].y);
			*/
			closesPt = findCloserPoint(Result, cart_points[coordinateCounter+1]);
			angleE = computeAngleE(closesPt, robotPoint, th); 
			vel_msg.angular.z = 2.25*angleE;
			rho =  sqrt(pow((robotPoint.x-cart_points[coordinateCounter+1].x), 2) + 
			pow((robotPoint.y-cart_points[coordinateCounter+1].y), 2));
			/*
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\nResult p1: (" << Result.p1.x << ", " << Result.p1.y 
			<< "), p2: (" << Result.p2.x << ", " << Result.p2.y << ")");
			
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nCurrent Position: x-position: " << x
				<<", y-position:" << y
				<< ", angleE: " << angleE*180/M_PI
				<< ", rotation: " << th*180/M_PI
				<< ", coordinateCounter: " << coordinateCounter);
				*/
			if(th == angleE)
			{
				vel_msg.angular.z = 0;
			}
			stage_vel.publish(vel_msg);
			
			if(rho <= 0.15)
			{
				coordinateCounter++;
				/*
				ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nCurrent Position: x-position: " << x
				<<", y-position:" << y
				<< ", rotation: " << th
				<< ", angleE: " << angleE*180/M_PI
				<< ", rotation: " << th*180/M_PI
				<< ", coordinateCounter: " << coordinateCounter);
				*/
				
			}
		}
		else
		{

			Result = solveCircleLineQuad(cart_points[coordinateCounter].x, 
				cart_points[coordinateCounter+1].x, 
				cart_points[coordinateCounter].y, 
				cart_points[coordinateCounter+1].y);

			closesPt = findCloserPoint(Result, cart_points[coordinateCounter+1]);
			angleE = computeAngleE(closesPt, robotPoint, th); 
			/*
			if(coordinateCounter == 3)
			{
				ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nClosest Point: x: " << closesPt.x
				<<", y:" << closesPt.y
				<< ", angleE: " << angleE*180/M_PI);
			}
			*/
			vel_msg.angular.z = 3.5*angleE;
		
			rho =  sqrt(pow((robotPoint.x-cart_points[coordinateCounter+1].x), 2) + 
			pow((robotPoint.y-cart_points[coordinateCounter+1].y), 2));
			if(th == angleE)
			{
				vel_msg.angular.z = 0;
			}
			stage_vel.publish(vel_msg);
			if(rho <= 0.35)
			{
				coordinateCounter++;
				
				ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nCurrent Position: x-position: " << x
				<<", y-position:" << y
				<< ", rotation: " << th
				<< ", angleE: " << angleE
				<< ", coordinateCounter: " << coordinateCounter);
				
				//if(coordinateCounter >= WayPoints)
				if(coordinateCounter >= tri_sqr)
				{
					vel_msg.linear.x = 0;
					vel_msg.angular.z = 0;
					stage_vel.publish(vel_msg);
					coordinateCounter = -1;
					return 0;
				}
			}
		}
		
		////////////////////////////////////////////
	}
	//ros::spin();
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
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		//<< "\nx1: "<< x1 <<", x2: " << x2 <<", y1: " << y1 << ", y2: " << y2 << ", coordinateCounter: " << coordinateCounter);
	pair<point> result;
	
	if(y1 == y2)//we have a horizontal line
	{
		result.p1.y = y1;
		result.p2.y = y2;
		double d1 = pow(lookaheadRadius, 2) - pow(y1,2); 
		if(d1 > 0)
		{
			result.p1.x = x1;
			result.p2.x = x2;
		}
		return result;
	}
	
	if(x1 == x2)//we have a vertical line
	{
		result.p1.x = x1;
		result.p2.x = x2;
		double d2 = pow(lookaheadRadius, 2) - pow(x1,2); 
		if(d2 > 0)
		{
			result.p1.y = y1;
			result.p2.y = y2;
		}
		return result;
	}
	
	double m = getSlope_L(x1, x2, y1, y2);
	double B = getIntercept_L(x1, y1, m);
	double a = 1 + pow(m, 2);
	double b = 2*B*m;
	double c = pow(B, 2) - pow(lookaheadRadius, 2);
	
	if(a<0.0000001)    // ==0
	{
		if(b>0.0000001)  // !=0
		{
			result.p1.x=result.p2.x=-c/b;
		}
		else
			if(c>0.000001) {
				ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nNo solutions exception.");
				throw "no solutions";}
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
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\nNo real solutions exception.");
		throw "non-real solutions";
	}

	return result;
}

double computeAngleE(point point1, point robot_point, double currentTheta)
{
	return (atan2((point1.y - robot_point.y), (point1.x - robot_point.x)) - currentTheta);
}
void PathMessageReceived(const nav_msgs::Path& msg)
{
	if(rec_msg == false)
	{
		int nmbrOfWaypoints = msg.poses.size();
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nWayPoints: " << nmbrOfWaypoints);
		pathMsg = msg;
		/*************************/
		/*
		WayPoints = nmbrOfWaypoints+1;
		
		poses.resize(WayPoints);
		cart_points.resize(WayPoints);
		for(int i = 0; i < WayPoints; i++)
		{
			if(i == WayPoints)
			{
				cart_points[i].x = pathMsg.poses[0].pose.position.x;
				poses[i].pose.position.x = pathMsg.poses[0].pose.position.x;
			}
			else
			{
				cart_points[i].x = pathMsg.poses[i].pose.position.x;
				poses[i].pose.position.x = pathMsg.poses[i].pose.position.x;
			}
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\ncart_points: i: " << i
				<<", x: " << cart_points[i].x
				<< ", y: " << cart_points[i].y);
		}
		*/
		/**************************/
		//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			//<< "\nPath Message: " << pathMsg);
			
		rec_msg = true;
	}
}

point findCloserPoint(pair<point> pair_of_pts, point inspect)
{
	double rho1 = pow(pair_of_pts.p1.x - inspect.x,2) + pow(pair_of_pts.p1.y - inspect.y,2);
	double rho2 = pow(pair_of_pts.p2.x - inspect.x,2) + pow(pair_of_pts.p2.y - inspect.y,2);

	if(rho1 <= rho2)
		return pair_of_pts.p1;
	else
		return pair_of_pts.p2;
}
