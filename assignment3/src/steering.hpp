#ifndef STEERING

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <angles/angles.h>
#include <ros/topic.h>

#define STEERING
const int spinning_rt = 1000;

class Steering
{
	private:
		//nodeHandle
		ros::NodeHandle s_nh;
		
		//publisher
		ros::Publisher s_pub;
		
		//subscribers
		ros::Subscriber s_sub; 
        ros::Subscriber s_getGoal;
             
		ros::Timer s_timer;
		
        geometry_msgs::Twist Msg;
		
		nav_msgs::Odometry odom_msg;
		
		float xPos, yPos, thetaPos;
		 
        float goal_xPos, goal_yPos, goal_thetaPos;
		
		bool move;
		
	public:
		
		Steering(ros::NodeHandle nh);
		
		void initSteering();
		
		ros::NodeHandle getNH();
		
		ros::Publisher get_publisher();
		
		void setMove(bool mv);
		
		float getX();
		
		float getY();
		
		float getTheta();
		
		void setLoc(float x, float y, float th);
		
		bool getMove();
		
		//Movement functions
		void moveDist(double distance, bool forward);

		void rotateDegrees(float degrees);
		
		void stopMoving();
		
		void moveAtSpeed(double speed, double time);//top speed is between 1.0 and -1.0
		
		void rotateAtSpeed(double speed, double time);//speed is in degrees per second, and angle can be between pi and -pi
		
		void moveAndRotate(double movespeed, double rotationspeed);
		//end of movement functions
		
		geometry_msgs::Twist getMsg();
		
		void positionMessageReceived(const nav_msgs::Odometry& msg);
		
		float get_goal_Xpos();
        
        float get_goal_Ypos();
        
        float get_theta_Pos();
        
        void setGoal(float x, float y, float theta);
 
        void goalMessage(const geometry_msgs::PoseStamped& msg);
        
        void stopMovingWithTimer(const ros::TimerEvent& event);
        
		double findRotatingAngle(float goalAngle);
        
        //void PointAndShoot(const geometry_msgs::PoseStamped& msg);
        void PointAndShoot();
        geometry_msgs::PoseStamped msg_pose;
        
        void pointToGoal();
        
        void Servoing();
        
        void ServoingAlternative();

};





#endif

