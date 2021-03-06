#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <math.h>
#include <iostream>
#include <exception>
#include <nav_msgs/Path.h>
#include <vector>

//Variables
//const double lookaheadRadius = sqrt(64.0);
double lookaheadRadius;
nav_msgs::Path pathMsg;
struct point {double x; double y;};
template<typename T> struct pair{ T p1; T p2; };
int tri_sqr;
std::vector<geometry_msgs::PoseStamped> poses;
std::vector<point> cart_points;
bool rec_msg = false;
geometry_msgs::Twist vel_msg;
int coordinateCounter = -1;
const double EPSILON = 0.0009;

//Functions
double getSlope_L(double x1, double x2, double y1, double y2);//returns the slope of a line, constructed from 2 given points
double getIntercept_L(double x, double y, double slope);//returns the y-intercept value of a line, given a point and slope
pair<point> solveCircleLineQuad(double x1, double x2, double y1, double y2, point robPt);//Solve for intersection points of the line L defined by 
//2 points and the circle C with radius defined by lookaheadRadius
double computeAngleE(point point1, point point2, double currentTheta);
void PathMessageReceived(const nav_msgs::Path& msg);
point findCloserPoint(pair<point> pair_of_pts, point inspect);
bool CompareDoubles(double A, double B);
void GetParam(std::string paramName, double* value);

int main(int argc, char** argv){
	ros::init(argc, argv, "local_planner");

	ros::NodeHandle nh;

	GetParam("lookaheadRadius", &lookaheadRadius);
 
	ros::Publisher stage_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Subscriber path_listener = nh.subscribe("/plan", 10, &PathMessageReceived);
	
	tf::TransformListener listener;
	tf::TransformListener pose_listener;//For getting current position
	
	ros::Rate rate(1000.0);
	while (nh.ok())
	{
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
		nav_msgs::Path Msg = *(ros::topic::waitForMessage<nav_msgs::Path>("/plan", nh));
		
		if(!rec_msg)
		{
			ros::spinOnce();
		}
		else
		{
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
			
			
			float deltaX = transform.getOrigin().x() - position_transform.getOrigin().x();
			float deltaY = transform.getOrigin().y() - position_transform.getOrigin().y();
			
			point robotPoint;
			robotPoint.x = x;
			robotPoint.y = y;
			float rho;
			pair<point> Result;	
			point closesPt;
			double angleE;
			vel_msg.linear.x = 0.1;
			if(coordinateCounter == -1)
			{
				Result = solveCircleLineQuad(robotPoint.x, 
					cart_points[coordinateCounter+1].x, robotPoint.y, cart_points[coordinateCounter+1].y,
					robotPoint);
					
				closesPt = findCloserPoint(Result, cart_points[coordinateCounter+1]);
				angleE = computeAngleE(closesPt, robotPoint, th); 
				rho =  sqrt(pow((robotPoint.x-cart_points[coordinateCounter+1].x), 2) + 
				pow((robotPoint.y-cart_points[coordinateCounter+1].y), 2));
				vel_msg.angular.z = 3.5*angleE;
				vel_msg.linear.x += (rho*0.65/lookaheadRadius);
				//if(abs(th-angleE) < 0.005)
				if(th == angleE)
				{
					vel_msg.angular.z = 0;
				}
				stage_vel.publish(vel_msg);
				
				if(rho <= 0.30)
				{
					coordinateCounter++;
					/*
					ROS_INFO_STREAM(std::setprecision(2) << std::fixed
					<< "\nCurrent Position: x-position: " << x
					<<", y-position:" << y
					<< ", rotation: " << th
					<< ", angleE: " << angleE*180/M_PI
					<< ", rotation: " << th*180/M_PI);
					*/
				}
			}
			else
			{

				Result = solveCircleLineQuad(cart_points[coordinateCounter].x, 
					cart_points[coordinateCounter+1].x, 
					cart_points[coordinateCounter].y, 
					cart_points[coordinateCounter+1].y, robotPoint);
				
				closesPt = findCloserPoint(Result, cart_points[coordinateCounter+1]);
				
				angleE = computeAngleE(closesPt, robotPoint, th); 
				
				rho =  sqrt(pow((robotPoint.x-cart_points[coordinateCounter+1].x), 2) + 
				pow((robotPoint.y-cart_points[coordinateCounter+1].y), 2));
				vel_msg.angular.z = 3.5*angleE;
				vel_msg.linear.x += (rho*0.65/lookaheadRadius);
				
				if(th == angleE)
				{
					vel_msg.angular.z = 0;
				}
				stage_vel.publish(vel_msg);
				if(rho <= 0.45)
				{
					coordinateCounter++;
					/*
					ROS_INFO_STREAM(std::setprecision(2) << std::fixed
					<< "\nCurrent Position: x-position: " << x
					<<", y-position:" << y
					<< ", rotation: " << th
					<< ", angleE: " << angleE
					<< ", coordinateCounter: " << coordinateCounter);
					*/
					if(coordinateCounter > tri_sqr)
					{
						return 0;
					}
				}
			}
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

pair<point> solveCircleLineQuad(double x1, double x2, double y1, double y2, point robPt)
{
	pair<point> result;
	
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		//<< "\nx1: " << x1 << ", x2: " << x2 << ", y1: " << y1 << ", y2: " << y2);
	
	if(CompareDoubles(y1,y2))//we have a horizontal line
	{
		result.p1.y = y1;
		result.p2.y = y2;
		double k2pos = sqrt((lookaheadRadius*lookaheadRadius) - pow((robPt.y - y1),2));
		double k2neg = -sqrt((lookaheadRadius*lookaheadRadius) - pow((robPt.y - y1),2));
		double d1 = pow(lookaheadRadius, 2) - pow(robPt.y-y1,2);//must be within lookahead radius 
		if(d1 > 0)
		{
			result.p1.x = x1 + k2pos;
			result.p2.x = x1 + k2neg;
		}
		return result;
	}
	
	if(CompareDoubles(x1,x2))//we have a vertical line
	{
		result.p1.x = x1;
		result.p2.x = x2;
		double k2pos = sqrt((lookaheadRadius*lookaheadRadius) - pow((robPt.x - x1),2));
		double k2neg = -sqrt((lookaheadRadius*lookaheadRadius) - pow((robPt.x - x1),2));
		double d2 = pow(lookaheadRadius, 2) - pow(robPt.x-x1,2);//must be within lookahead radius 
		if(d2 > 0)
		{
			result.p1.y = k2pos + y1;
			result.p2.y = k2neg + y1;			
		}
		return result;
	}
	
	double k = robPt.x;
	double h = robPt.y;
	double m = getSlope_L(x1, x2, y1, y2);
	double B = getIntercept_L(x1, y1, m);
	double a = 1 + pow(m, 2);
	double b = 2*B*m -2*k -2*m*h;
	double c = h*h + k*k + pow(B, 2) - 2*B*h - pow(lookaheadRadius, 2);
	
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
	double angle = (atan2((point1.y - robot_point.y), (point1.x - robot_point.x)) - currentTheta);
	
	if(angle > M_PI)
	{
		angle -= 2*M_PI;
	}
	else if( angle < (-1*M_PI))
	{
		angle += 2*M_PI;
	} 
	
	return angle;
}
void PathMessageReceived(const nav_msgs::Path& msg)
{
	if(rec_msg == false)
	{
		int nmbrOfWaypoints = msg.poses.size();
		
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\nTotal WayPoints: " << nmbrOfWaypoints);
				
		pathMsg = msg;
		
		tri_sqr = nmbrOfWaypoints;
		
		poses.resize(tri_sqr);
		cart_points.resize(tri_sqr);
		for(int i = 0; i < tri_sqr; i++)
		{
			cart_points[i].x = pathMsg.poses[i].pose.position.x;
			poses[i].pose.position.x = pathMsg.poses[i].pose.position.x;
			cart_points[i].y = pathMsg.poses[i].pose.position.y;
			poses[i].pose.position.y = pathMsg.poses[i].pose.position.y;
			
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
				<< "\ncart_points: i: " << i
				<<", x: " << cart_points[i].x
				<< ", y: " << cart_points[i].y);
		}
		
		/**********************************************/
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

bool CompareDoubles(double A, double B) 
{
   double diff = std::abs(A - B);//be sure to use abs provided by std, 
   //These convenience abs overloads are exclusive of C++. In C, abs is only 
   //declared in <stdlib.h> (and operates on int values).
   return (diff < EPSILON);
}
void GetParam(std::string paramName, double* value)
{
	const std::string PARAM_NAME = paramName;
	bool ok = ros::param::get(PARAM_NAME, *value);
	if (!ok)
	{
		ROS_FATAL_STREAM("Could not get parameter " << PARAM_NAME);
		exit(1);
	}
	ROS_INFO_STREAM(
			std::setprecision(2) << std::fixed << "\n" <<paramName << ": " << *value);
}

