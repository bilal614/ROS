#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include "steering.hpp"
#include <math.h>
#include <iostream>
#include <exception>
#include <nav_msgs/Path.h>

/**Global Variables**/
const double lookaheadRadius = sqrt(64.0);
nav_msgs::Path pathMsg;
struct point {double x; double y;};
template<typename T> struct pair{ T p1; T p2; };
//const int tri_sqr = 4;
int WayPoints = 0;
//geometry_msgs::PoseStamped poses[(tri_sqr+1)];
//point cart_points[(tri_sqr+1)];
tf::StampedTransform transform;
tf::StampedTransform position_transform;
bool msg_received_and_executed = false;


/**Functions**/
double getSlope_L(double x1, double x2, double y1, double y2);//returns the slope of a line, constructed from 2 given points
double getIntercept_L(double x, double y, double slope);//returns the y-intercept value of a line, given a point and slope
pair<point> solveCircleLineQuad(double x1, double x2, double y1, double y2);//Solve for intersection points of the line L defined by 
//2 points and the circle C with radius defined by lookaheadRadius
double computeAngleE(point point1, point point2, double currentTheta);
void PathMessageReceived(const nav_msgs::Path& msg);
point findCloserPoint(pair<point> pair_of_pts, point inspect);
int coordinateCounter = -1;
int followWaypoints(ros::Publisher stage_vel, geometry_msgs::Twist vel_msg,  int nr_of_waypoints);



int main(int argc, char** argv){
	ros::init(argc, argv, "local_planner");

	ros::NodeHandle nh;	
	
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\nPath Message: " << pathMsg);

	ros::Publisher stage_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	
	ros::Subscriber path_listener = nh.subscribe("/plan", 1000, &PathMessageReceived);
	
	tf::TransformListener listener;
	tf::TransformListener pose_listener;//For getting current position
	
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
		
		/*
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\nCurrent Position: x-position: " << x
			<<", y-position:" << y
			<< ", rotation: " << th);
			*/

		geometry_msgs::Twist vel_msg;
		//nav_msgs::Path Msg = *(ros::topic::waitForMessage<nav_msgs::Path>("/plan", ros::Duration(0.25)));
		if(msg_received_and_executed)
		{
			followWaypoints(stage_vel, vel_msg, WayPoints);
		}
		
		rate.sleep();
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
			if(c>0.000001) throw "no solutions";
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
void PathMessageReceived(const nav_msgs::Path& msg)
{
	int nmbrOfWaypoints = sizeof(msg.poses)/sizeof(msg.poses[0]);
	pathMsg = msg;
	msg_received_and_executed = true;
	WayPoints = nmbrOfWaypoints;
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

int followWaypoints(ros::Publisher stage_vel, geometry_msgs::Twist vel_msg,  int nr_of_waypoints)
{
	geometry_msgs::PoseStamped poses[nr_of_waypoints+1];
	point cart_points[nr_of_waypoints+1];
	
	for(int i = 0; i < nr_of_waypoints; i++)
	{
		cart_points[i].x = pathMsg.poses[i].pose.position.x;
		poses[i].pose.position.x = pathMsg.poses[i].pose.position.x;
	}
	//otherwise get them from global poses and cart_points
	/*
	geometry_msgs::PoseStamped poses[nr_of_waypoints+1];
	point cart_points[nr_of_waypoints+1];
	//fill out waypoints 
	for (int i = 0; i < nr_of_waypoints+1; i++)
	{
		//cart_points[i].x = 2*(i+1);
		//poses[i].pose.position.x = 2*(i+1);//(2,4,6)
		cart_points[i].x = (i%2)*3;
		poses[i].pose.position.x = (i%2)*3;//(0,3,0,3)
		
		//if(i == 1)
		if(i > 1 && i < 4)
		{
			//poses[i].pose.position.y = -6;
			//cart_points[i].y = -6;
			poses[i].pose.position.y = 4;
			cart_points[i].y = 4;
		}
		else
		{
			//poses[i].pose.position.y = -2;//(-2,-6,-2)
			//cart_points[i].y = -2;
			poses[i].pose.position.y = 0;//(-2,-6,-2)
			cart_points[i].y = 0; 
		}//(2,-2) (4,-6) (6,-2)
		//if(i == 3)
		if(i == 4)
		{
			/*
			cart_points[i].x = 2;
			poses[i].pose.position.x = 2;
			cart_points[i].y = -2;
			poses[i].pose.position.y = -2;
			*/
			/*
			cart_points[i].x = 0;
			poses[i].pose.position.x = 0;
			cart_points[i].y = 0;
			poses[i].pose.position.y = 0;
		}		
	}
	*/
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
	vel_msg.linear.x = 0.35;
	if(coordinateCounter == -1)
	{
		Result = solveCircleLineQuad(robotPoint.x, 
			cart_points[coordinateCounter+1].x, robotPoint.y, 
			cart_points[coordinateCounter+1].y);
		
		closesPt = findCloserPoint(Result, cart_points[coordinateCounter+1]);
		angleE = computeAngleE(closesPt, robotPoint, th); 
		vel_msg.angular.z = 1.25*angleE;
		rho =  sqrt(pow((robotPoint.x-cart_points[coordinateCounter+1].x), 2) + 
		pow((robotPoint.y-cart_points[coordinateCounter+1].y), 2));
		
		if(th == angleE)
		{
			vel_msg.angular.z = 0;
		}
		stage_vel.publish(vel_msg);
		
		if(rho <= 0.15)
		{
			coordinateCounter++;
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\nCurrent Position: x-position: " << x
			<<", y-position:" << y
			<< ", rotation: " << th
			<< ", angleE: " << angleE*180/M_PI
			<< ", rotation: " << th*180/M_PI
			<< ", coordinateCounter: " << coordinateCounter);
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
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "\nClosest Point: x: " << closesPt.x
		<<", y:" << closesPt.y
		<< ", angleE: " << angleE*180/M_PI);
		*/
		vel_msg.angular.z = 3.0*angleE;
	
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
			if(coordinateCounter >= nr_of_waypoints)
			{
				vel_msg.linear.x = 0;
				vel_msg.angular.z = 0;
				stage_vel.publish(vel_msg);
				coordinateCounter = -1;
				msg_received_and_executed = false;
				return coordinateCounter;
			}
		}
	}
	return -1;
}

