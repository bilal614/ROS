#include "traveler.hpp"

void Sender::poseMessageReceived(const turtlesim::Pose& msg)
{
	/*
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "position=(" << msg.x << ", " << msg.y << ")"
		<< " direction=" << msg.theta);
	*/
	this->setLoc(msg.x, msg.y, msg.theta);
}

Sender::Sender(ros::NodeHandle nh)
{
	sideL = 0.0;
	
	s_nh = nh;
	
	s_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
	
	rotationCount = 0;
	
	move = false;
	
	Msg.linear.y = 0;
	Msg.linear.y = 0;
	Msg.linear.z = 0;
	
	Msg.angular.x = 0;
	Msg.angular.y = 0;
	Msg.angular.z = 0;
	
}

ros::NodeHandle Sender::getNH()
{
	return s_nh;
}

double Sender::getSide()
{
	return sideL;
}
		
void Sender::setSide(double s)
{
	sideL = s;
}

ros::Publisher Sender::get_publisher()
{
	return s_pub;
}

void Sender::setMove(bool mv)
{
	move = mv;
}

bool Sender::getMove()
{
	return move;
}

geometry_msgs::Twist Sender::getMsg()
{
	return Msg;
}

/*
void Sender::getLoc(float *x, float *y, float *th);
{
	*x = xPos;
	*y = yPos;
	*th = thetaPos; 
}
*/

float Sender::getTheta()
{
	return thetaPos;
}

float Sender::getX()
{
	return xPos;
}

float Sender::getY()
{
	return yPos;
}
	
void Sender::setLoc(float x, float y, float th)
{
	xPos = x;
	yPos = y;
	thetaPos = th;
}

void Sender::travelSideLength(double distance, bool forward)
{
	geometry_msgs::Twist vel_msg = getMsg();
	
	this->setMove(true);
	
	float originalPosition = this->getX() + this->getY();
	
	double speed = distance*0.25f;
	
	if(forward)
	{
		vel_msg.linear.x = speed;
	}
	else
	{
		vel_msg.linear.x = -1*speed;
	}
	
	ros::Rate rate(1000); //@ 2 Hz
	
	
	//try this
	ros::Subscriber s_sub = (this->getNH()).subscribe("turtle1/pose", 1000, &Sender::poseMessageReceived, this);
	
	s_timer = s_nh.createTimer(ros::Duration(3.0f), &Sender::stopMoving, this, true);
	
	
	float currentX = this->getX();
	
	while(this->getMove())
	{
		currentX = this->getX();
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "X-position = " << currentX);
		if(!ros::ok())
		{
			break;
		}
		(this->get_publisher()).publish(vel_msg);
		
		if( abs(currentX - originalPosition) < this->getSide())
		{
			(this->get_publisher()).publish(vel_msg);
		}
		
		rate.sleep();//wait until time for another iteration
		ros::spinOnce();
	}
}

void Sender::rotate60Degrees(bool cw)
{
	float delta_time = 1.0f;
	
	float angular_speed = (deg120/delta_time)*(2.0/3.0);
	
	geometry_msgs::Twist vel_msg = getMsg();
	
	this->setMove(true);
	if(cw)
	{
		vel_msg.angular.z = angular_speed;
	}
	else
	{
		vel_msg.angular.z = -1*angular_speed;
	}
	
	ros::Rate rate(1000); //@ 2 Hz
	
	//try this
	ros::Subscriber s_sub = (this->getNH()).subscribe("turtle1/pose", 1000, &Sender::poseMessageReceived, this);
	
	s_timer = s_nh.createTimer(ros::Duration(3.0f), &Sender::stopMoving, this, true);
	
	float Theta = this->getTheta();
	
	while(this->getMove())
	{
		Theta = this->getTheta();
		//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		//<< "theta = " << Theta);
		if(!ros::ok())
		{
			break;
		}
		
		(this->get_publisher()).publish(vel_msg);
		rate.sleep();//wait until time for another iteration
		
		if(Theta != 0.000f || Theta != (M_PI*(2/3)) || Theta != (M_PI*(4/3)))
		{
			(this->get_publisher()).publish(vel_msg);
		}
		
		ros::spinOnce();
	}
	
}

void Sender::stopMoving(const ros::TimerEvent& event)
{
	//float Theta = this->getTheta();
	geometry_msgs::Twist vel_msg = getMsg();
	
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed
	//<< "rotation count = " << rotationCount);	
	
	//if(Theta == 0.000f || Theta == (M_PI*(2/3)) || Theta == (M_PI*(4/3))) // && this->getMove() == false
	//{
		this->setMove(false);
		vel_msg.linear.x = 0;
		vel_msg.angular.z = 0;
		rotationCount++;
		/*
		if(rotationCount%2 == 0)
		{
			rotationCount = 0;
		}
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "rotation count = " << rotationCount);
		*/
		(this->get_publisher()).publish(vel_msg);
	/*
	}
	else if(rotationCount == 0)
	{
		rotationCount = 1;
		(this->get_publisher()).publish(vel_msg);
	}
	*/
}

void Sender::triangleMessage(const assignment1::Triangle& msg)
{
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "Message: " << msg);
	//this->adjustSideLength(msg.sideLength);
	/*
	double sL;
	if(msg.sideLength > abs(msg.sideLength - 3.5f))
	{
		sL = abs(msg.sideLength - 3.5f);
	}
	else
	{
		sL = msg.sideLength; 
	}
	*/
	if(msg.cw > 0)
	{	
		this->makeTriangle(msg.sideLength, true);
	}
	else
	{
		this->makeTriangle(msg.sideLength, false);
	}
}

void Sender::adjustSideLength(double l)
{
	ros::Subscriber s_sub = (this->getNH()).subscribe("turtle1/pose", 1000, &Sender::poseMessageReceived, this);
	
	if(this->getTheta() > 0.000f && this->getTheta() < M_PI_2)
	{
		if(l > this->getX()  || l > this->getY())
		{
			double tempSide = std::min(this->getX(), this->getY());
			this->setSide(tempSide);	
		}
	}
	
	if(this->getTheta() > M_PI_2)
	{
		if(l > (11.00f - this->getX())  || l > (11.00f - this->getY()))
		{
			double tempSide = std::min((this->getX() - 11.00f), (this->getY() - 11.00f));
			this->setSide(tempSide);	
		}
	}
	ros::spin();
}

void Sender::makeTriangle(double sideLength, bool orientation)
{
	
	this->travelSideLength(sideLength, true);
	
	ros::Duration(0.5).sleep();
	
	this->rotate60Degrees(orientation);
	
	ros::Duration(0.5).sleep();
	
	this->travelSideLength(sideLength, true);
	
	ros::Duration(0.5).sleep();
	
	this->rotate60Degrees(orientation);
	
	ros::Duration(0.5).sleep();

	this->travelSideLength(sideLength, true);
	
	ros::Duration(0.5).sleep();	
}
