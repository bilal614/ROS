#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include "steering.hpp"
#include <math.h>
#include <iostream>
#include <exception>
#include <nav_msgs/Path.h>


const double lookaheadRadius = sqrt(64.0);
nav_msgs::Path pathMsg;
struct point {double x; double y;};
template<typename T> struct pair{ T p1; T p2; };

double getSlope_L(double x1, double x2, double y1, double y2);//returns the slope of a line, constructed from 2 given points
double getIntercept_L(double x, double y, double slope);//returns the y-intercept value of a line, given a point and slope
pair<point> solveCircleLineQuad(double x1, double x2, double y1, double y2);//Solve for intersection points of the line L defined by 
//2 points and the circle C with radius defined by lookaheadRadius
double computeAngleE(point point1, point point2, double currentTheta);
void PathMessageReceived(const nav_msgs::Path& msg);
point findCloserPoint(pair<point> pair_of_pts, point inspect);
int coordinateCounter = -1;

int main(int argc, char** argv){
	ros::init(argc, argv, "local_planner");

	ros::NodeHandle nh;
  
	geometry_msgs::PoseStamped poses[4];
	point cart_points[4];
	
	for (int i = 0; i < 4; i++)
	{
		//point temp_point;
		cart_points[i].x = ((i%2)+2)*2-2;
		poses[i].pose.position.x = ((i%2)+2)*2-2;//(2,4,2,4) (2,2) (4,2) (2,0) (4,0)
		if(i < 2)
		{
			poses[i].pose.position.y = 2;//(2,2,0,0)
			cart_points[i].y = 2; 
		}
		else
		{
			poses[i].pose.position.y = 0;
			cart_points[i].y = 0;
		}
			
	} 
	
	
	std::vector<geometry_msgs::PoseStamped> vec(poses, poses + sizeof poses / sizeof poses[0]);
	
	pathMsg.poses = vec;
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\nPath Message: " << pathMsg);

	ros::Publisher stage_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	
	tf::TransformListener listener;//Listening to Goal related transformation between parent map and child odom
	tf::TransformListener pose_listener;//For current position

	ros::Rate rate(1000.0);
	while (nh.ok())
	{
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
		
		//x and y represent current position of the robot, th represents orientation of the robot	
		double x = position_transform.getOrigin().x();
		double y = position_transform.getOrigin().y();
		double th = tf::getYaw(position_transform.getRotation());
		/*
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\nCurrent Position: x-position: " << x
			<<", y-position:" << y
			<< ", rotation: " << th);
			*/
		
		//steering.setGoal(transform.getOrigin().x(), transform.getOrigin().y(), M_PI);
		geometry_msgs::Twist vel_msg;
		
		float deltaX = transform.getOrigin().x() - position_transform.getOrigin().x();
		float deltaY = transform.getOrigin().y() - position_transform.getOrigin().y();
		
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
				cart_points[coordinateCounter+1].x, robotPoint.y, cart_points[coordinateCounter+1].y);
				
			closesPt = findCloserPoint(Result, cart_points[coordinateCounter+1]);
			angleE = computeAngleE(closesPt, robotPoint, th); 
			vel_msg.angular.z = 1.0*angleE;
			rho =  sqrt(pow((robotPoint.x-cart_points[coordinateCounter+1].x), 2) + 
			pow((robotPoint.y-cart_points[coordinateCounter+1].y), 2));
			//if(abs(th-angleE) < 0.005)
			if(th == angleE)
			{
				vel_msg.angular.z = 0;
			}
			stage_vel.publish(vel_msg);
			
			//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				//<< "\nRho: " << rho);
			if(rho <= 0.15)
			{
				ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nCurrent Position: x-position: " << x
				<<", y-position:" << y
				<< ", rotation: " << th
				<< ", angleE: " << angleE*180/M_PI
				<< ", rotation: " << th*180/M_PI);
				coordinateCounter++;
			}
		}
		else
		{

			Result = solveCircleLineQuad(cart_points[coordinateCounter].x, 
				cart_points[coordinateCounter+1].x, cart_points[coordinateCounter].y, cart_points[coordinateCounter+1].y);
			//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			//<< "\nResult: p1: (" << Result.p1.x << ", " << Result.p1.y << "), p2: (" 
			//<< Result.p2.x << ", " << Result.p2.y << ")");	
			closesPt = findCloserPoint(Result, cart_points[coordinateCounter+1]);
			closesPt.x += robotPoint.x;
			closesPt.y += robotPoint.y;
			angleE = computeAngleE(closesPt, robotPoint, th); 
			
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "\nClosest Point: x: " << closesPt.x
			<<", y:" << closesPt.y
			<< ", angleE: " << angleE*180/M_PI);
			
			vel_msg.angular.z = 1.5*angleE;
			//if(abs(th-angleE) < 0.005)
			rho =  sqrt(pow((robotPoint.x-cart_points[coordinateCounter+1].x), 2) + 
			pow((robotPoint.y-cart_points[coordinateCounter+1].y), 2));
			if(th == angleE)
			{
				vel_msg.angular.z = 0;
			}
			stage_vel.publish(vel_msg);
			if(rho <= 0.25)
			{
				ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nCurrent Position: x-position: " << x
				<<", y-position:" << y
				<< ", rotation: " << th
				<< ", angleE: " << angleE);
				coordinateCounter++;
				if(coordinateCounter > 3)
				{
					return 0;
				}
			}
		}
		
		//testing
		
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
			result.p1.x = sqrt(d1);
			result.p2.x = -sqrt(d1);
			/*
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
					<< "\nResult p1: (" << result.p1.x << ", " << result.p1.y << "), p2: ("
					<< result.p2.x << ", " << result.p2.y << ")");
				*/	
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
			result.p1.y = sqrt(d2);
			result.p2.y = -sqrt(d2);
			/*
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nResult p1: (" << result.p1.x << ", " << result.p1.y << "), p2: ("
				<< result.p2.x << ", " << result.p2.y << ")");
				*/
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
	
}

point findCloserPoint(pair<point> pair_of_pts, point inspect)
{
	double rho1 = pow(pair_of_pts.p1.x - inspect.x,2) + pow(pair_of_pts.p1.y - inspect.y,2);
	double rho2 = pow(pair_of_pts.p2.x - inspect.x,2) + pow(pair_of_pts.p2.y - inspect.y,2);
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
					<< "\nrho1 = " << rho1
					<< ", rho2 = " << rho2);
	if(rho1 <= rho2)
		return pair_of_pts.p1;
	else
		return pair_of_pts.p2;
}
/*
bool goToPoint(point point1, point robot_point, double currentTheta)
{
	
}
*/

/*
		bool reached = false;
		float rho; 
		for(int i = 0; i < 3; i++)
		{
			while(!reached)
			{
				th = tf::getYaw(position_transform.getRotation());
				point robotPoint;
				robotPoint.x = x;
				robotPoint.y = y;
				rho =  sqrt(pow((robotPoint.x-cart_points[i+1].x), 2) + pow((robotPoint.y-cart_points[i+1].y), 2));

				pair<point> Result = solveCircleLineQuad(cart_points[i].x, cart_points[i+1].x, cart_points[i].y, cart_points[i+1].y);
				point closesPt = findCloserPoint(Result, cart_points[i+1]);
				double angleE = computeAngleE(closesPt, robotPoint, th); 
				vel_msg.angular.z = 0.75*angleE;
				stage_vel.publish(vel_msg);
				
				if(th == angleE)
				{
					vel_msg.angular.z = 0;
				}
				
				ROS_INFO_STREAM(std::setprecision(2) << std::fixed
					<< "\nPoint1: x = " << Result.p1.x 
					<< ", y = " << Result.p1.y
					<< "\nPoint2: x = " << Result.p2.x 
					<< ", y = " << Result.p2.y
					<< ", angle e = " << angleE
					<< ", theta = " << th);
					
				/*
				if(rho < 0.15)
				{
					if(i == 3)
					{
						reached = true;
						vel_msg.angular.z = 0;
						vel_msg.linear.x = 0;
						stage_vel.publish(vel_msg);
					}
					break;
				}
			}
		}
		*/
		
		/* testing
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
			<< "\nAngle e: " << angleE*180/M_PI
			<< ", robot point: x: " << robotPoint.x
			<< ", y: " << robotPoint.y);
			
		for(int i = 0; i < 4; i++)
		{
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nSquare Points i(" << i << "): x: " << cart_points[i].x
				<<", y:" << cart_points[i].y);
		}
		*/
