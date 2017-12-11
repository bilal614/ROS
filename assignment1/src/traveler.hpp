#ifndef TRAVELER

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include "assignment1/Triangle.h"


#define TRAVELER

#define TIMER_TIMEOUT (0.5F) //float point in seconds

const float deg120 = (M_PI/3);

struct location
{
	float x;
	float y;
	float theta;
};

class Sender
{
	private:
		ros::NodeHandle s_nh;
		
		ros::Publisher s_pub;
		
		//ros::Subscriber s_sub;
		
		ros::Timer s_timer;
		
		geometry_msgs::Twist Msg;
		
		float xPos, yPos, thetaPos;
		
		double sideL;
		
		bool move;
		
	public:
	
		int rotationCount;
		
		Sender(ros::NodeHandle nh);
		
		ros::NodeHandle getNH();
		
		ros::Publisher get_publisher();
		
		void setMove(bool mv);
		
		void adjustSideLength(double l);
		
		//void getLoc(float *x, float *y, float *th);
		float getX();
		
		float getY();
		
		float getTheta();
		
		void setLoc(float x, float y, float th);
		
		bool getMove();
		
		double getSide();
		
		void setSide(double s);
		
		void travelSideLength(double distance, bool forward);

		void rotate60Degrees(bool cw);
		
		void stopMoving(const ros::TimerEvent& event);
		
		geometry_msgs::Twist getMsg();
		
		void makeTriangle(double sideLength, bool orientation);
		
		void poseMessageReceived(const turtlesim::Pose& msg);
		
		void triangleMessage(const assignment1::Triangle& msg);
};


#endif

