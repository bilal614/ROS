#include "steering.hpp"

void Steering::positionMessageReceived(const nav_msgs::Odometry& msg)
{
	/*
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "\nOdometry Message: x-position: " << msg.pose.pose.position.x
		<<", y-position:" << msg.pose.pose.position.y
		<<", z-theta:" << tf::getYaw(msg.pose.pose.orientation));
	*/
	this->setLoc(msg.pose.pose.position.x, 
		msg.pose.pose.position.y, 
		tf::getYaw(msg.pose.pose.orientation));
		//msg.pose.pose.orientation.z);
}



Steering::Steering(ros::NodeHandle nh)
{
	s_nh = nh;
	
	s_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
		
	move = false;
	
	Msg.linear.y = 0;
	Msg.linear.y = 0;
	Msg.linear.z = 0;
	
	Msg.angular.x = 0;
	Msg.angular.y = 0;
	Msg.angular.z = 0;

	/*
	goal_xPos = 0.0;
	goal_yPos = 0.0;
	goal_thetaPos = 0.0;
	*/
	s_sub = nh.subscribe("/odom", 1000, &Steering::positionMessageReceived, this);
	s_getGoal = nh.subscribe("/goal", 100, &Steering::goalMessage, this);
	ros::spinOnce();
	ros::Duration(0.5).sleep();
}

void Steering::initSteering()
{
	s_sub = s_nh.subscribe("/odom", 1000, &Steering::positionMessageReceived, this);
	ros::Duration(0.5).sleep();
	s_getGoal = s_nh.subscribe("/goal", 1, &Steering::goalMessage, this);
}

ros::NodeHandle Steering::getNH()
{
	return s_nh;
}

void Steering::setLoc(float x, float y, float th)
{
	xPos = x;
	yPos = y;
	if(th >= 0)
	{
		thetaPos = th;
	}
	else
	{
		thetaPos = th + 2*M_PI;
	}
}

void Steering::moveDist(double distance, bool forward)
{
	geometry_msgs::Twist vel_msg = getMsg();
	
	this->setMove(true);
	
	double speed = distance;
	
	if(forward)
	{
		vel_msg.linear.x = speed;
	}
	else
	{
		vel_msg.linear.x = -1*speed;
	}
		
	float deltaX, deltaY; 
	float originalX = this->getX();
	float originalY = this->getY();
	float currentTheta = this->getTheta();
	float targetX, targetY;
	targetX = distance*cos(currentTheta);
	targetY = distance*sin(currentTheta);
	
	float originalPosition = hypot(originalX, originalY);
	float currentPosition = originalPosition;
	/*
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "Moving call bfr: " 
		<< ", currentPosition = " << currentPosition
		<< ", originalPosition = " << originalPosition
		<< ", distance = " << distance
		<< ", targetX = " << targetX
		<< ", targetY = " << targetY);
		*/
	
	s_timer = s_nh.createTimer(ros::Duration(static_cast<float>(speed)), &Steering::stopMovingWithTimer, this, true);
	
	ros::Rate rate(spinning_rt); //@ 1000 Hz
	float currentX, currentY;
	while(this->getMove())
	{
		deltaX = this->getX() - originalX;
		deltaY = this->getY() - originalY;
		currentPosition = hypot(deltaX, deltaY);
		currentX = this->getX();		
		currentY = this->getY();
		if(!ros::ok())
		{
			break;
		}
		(this->get_publisher()).publish(vel_msg);
		
		rate.sleep();//wait until time for another iteration
		ros::spinOnce();
	}
}


void Steering::stopMoving()
{
	/*
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "STOP MOVING FUNCTION WAS INVOKED\n"
		<< "current X: " << this->getX()
		<< ", current Y: " << this->getY()
		<< ", current Theta: " << this->getTheta()*180/M_PI);
		*/
	geometry_msgs::Twist vel_msg = getMsg();
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
		
	(this->get_publisher()).publish(vel_msg);
	this->setMove(false);
}

void Steering::stopMovingWithTimer(const ros::TimerEvent& event)
{
	/*
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "STOP MOVING FUNCTION WAS INVOKED\n"
		<< "current X: " << this->getX()
		<< ", current Y: " << this->getY()
		<< ", current Theta: " << this->getTheta()*180/M_PI);
		*/
	geometry_msgs::Twist vel_msg = getMsg();
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
		
	(this->get_publisher()).publish(vel_msg);
	this->setMove(false);
}

void Steering::rotateDegrees(float degrees)
{
	float originalTheta = this->getTheta();	
	float currentTheta = originalTheta;
	float degrees_temp = degrees;
	if(degrees_temp > 360)
	{
		degrees_temp = degrees - std::floor(degrees/360)*360.00f;
	}
	if(degrees_temp < -360)
	{
		degrees_temp = degrees + std::floor(degrees/360.00f)*360.00f;
	}	
	degrees_temp = degrees_temp*M_PI/180.00f;//this is in radians now
	float delta_theta = degrees_temp - originalTheta;
	float angular_speed;
	
	geometry_msgs::Twist vel_msg = getMsg();
	
	angular_speed = delta_theta/6.5f;
	
	this->setMove(true);
	/*
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
	<< "Rotation Call bfr:"
	<< ", currentTheta = " << currentTheta*180/M_PI  
	<< ", degrees = " << degrees 
	<< ", delta_theta= " << delta_theta
	<< ", angular_speed=  " << angular_speed );
	*/
	vel_msg.angular.z = angular_speed;
	
	//ros::Duration(0.5).sleep();
	
	s_timer = s_nh.createTimer(ros::Duration(6.5f), &Steering::stopMovingWithTimer, this, true);
	
	ros::Rate rate(spinning_rt); 
	
	while(this->getMove())
	{
		currentTheta = this->getTheta();
		
		if(!ros::ok())
		{
			break;
		}
		(this->get_publisher()).publish(vel_msg);
		rate.sleep();//wait until time for another iteration
		
		ros::spinOnce();
	}
	/*
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
	<< "Rotation Call after:"
	<< ", currentTheta = " << currentTheta*180/M_PI 
	<< ", degrees = " << degrees 
	<< ", delta_theta= " << delta_theta
	<< ", angular_speed=  " << angular_speed );
	*/
}

void Steering::moveAtSpeed(double speed, double time)
{
	geometry_msgs::Twist vel_msg = getMsg();
	
	this->setMove(true);//sets moving attribute to true
	
	if(speed > 1.0)
	{
		vel_msg.linear.x = 1.0;
	}
	else if(speed < -1.0)
	{
		vel_msg.linear.x = -1.0;
	}
	else
	{
		vel_msg.linear.x = speed;
	}
	float currentX = this->getX();
	float currentY = this->getY();
	(this->get_publisher()).publish(vel_msg);
	
	s_timer = s_nh.createTimer(ros::Duration(static_cast<float>(time)), &Steering::stopMovingWithTimer, this, true);
	
	ros::Rate rate(spinning_rt); //@ 1000 Hz
	
	while(this->getMove())
	{
		/*
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "Position Stats:"
		<< ", currentTheta = " << this->getTheta()*180/M_PI 
		<< ", current X = " << currentX 
		<< ", currentY = " << currentY);
		*/
		currentX = this->getX();		
		currentY = this->getY();
		if(!ros::ok())
		{
			break;
		}
		(this->get_publisher()).publish(vel_msg);
		
		rate.sleep();//wait until time for another iteration
		ros::spinOnce();
	}	
}
		
void Steering::rotateAtSpeed(double speed, double time)
{
	float currentTheta = this->getTheta();
	geometry_msgs::Twist vel_msg = getMsg();
	
	float speed_in_rad = angles::from_degrees(speed);	
	
	this->setMove(true);
	
	vel_msg.angular.z = speed_in_rad;
	(this->get_publisher()).publish(vel_msg);
	
	s_timer = s_nh.createTimer(ros::Duration(static_cast<float>(time)), &Steering::stopMovingWithTimer, this, true);
	
	ros::Rate rate(spinning_rt); 
	
	while(this->getMove())
	{
		currentTheta = this->getTheta();
		if(!ros::ok())
		{
			break;
		}
		(this->get_publisher()).publish(vel_msg);
		/*
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "Rotation:"
		<< ", currentTheta = " << currentTheta*180/M_PI 
		<< ", speed(in deg) = " << speed 
		<< ", time(in sec)= " << time
		<< ", angular_speed=  " << vel_msg.angular.z );
		*/
		rate.sleep();//wait until time for another iteration
		ros::spinOnce();
	}	
}
		
ros::Publisher Steering::get_publisher()
{
	return s_pub;
}

void Steering::setMove(bool mv)
{
	move = mv;
}

bool Steering::getMove()
{
	return move;
}

geometry_msgs::Twist Steering::getMsg()
{
	return Msg;
}

float Steering::getTheta()
{
	return thetaPos;
}

float Steering::getX()
{
	return xPos;
}

float Steering::getY()
{
	return yPos;
}

/*Get input as a PoseStamped message
 The out put should be x,y,z and Theta*/
/*for publishing the PoseStamped msg
 * --60 degree-- rostopic pub /goal geometry_msgs/PoseStamped '{ header: { frame_id: "/map" }, pose: { position: { x: 0.25, y: 0 }, orientation: { x: 0.752, y: 0.3, z: 0.326, w: 0.488 } } }'
 * --180 degree-- rostopic pub /goal geometry_msgs/PoseStamped '{ header: { frame_id: "/map" }, pose: { position: { x: 0.25, y: 0 }, orientation: { x: 0, y: 0, z: 1, w: 0 } } }'
 * */
void Steering::goalMessage(const geometry_msgs::PoseStamped& msg)
{
    tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    setGoal(msg.pose.position.x, msg.pose.position.y, yaw);   
    msg_pose = msg;
    
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed
                    << "goal_x = " << goal_xPos
                    << ", goal_y = " << goal_yPos
                    << ", goal_direction = " << goal_thetaPos
                    << ", goal_direction (Degree) = " << (goal_thetaPos*180)/M_PI
                    << ", yaw = " << yaw);
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
                    << "msg.pose.position.x = " << msg.pose.position.x
                    << ", msg.pose.position.y = " << msg.pose.position.y
                    << ", yaw = " << yaw
                    << ", yaw (Degree) = " << (yaw*180)/M_PI);
}

void Steering::setGoal(float x, float y, float theta)
{
	goal_xPos = x;
	goal_yPos = y;
	goal_thetaPos = theta;
}
 
float Steering::get_goal_Xpos()
{
    return goal_xPos;
}
 
float Steering::get_goal_Ypos()
{
    return goal_yPos;
}

float Steering::get_theta_Pos()
{
    return goal_thetaPos;
}

void Steering::pointToGoal()
{
    double goalDir;
    goalDir = atan2((goal_yPos - this->getY()), (goal_xPos - this->getX()));
    /*
    ROS_INFO_STREAM(std::setprecision(4) << std::fixed
    << "deltaY = " << (goal_yPos - this->getY())
    << ", deltaX = " << (goal_xPos - this->getX())
    << ", goal_yPos = " << goal_yPos
    << ", goal_xPos = " << goal_xPos
    << ", currentY = " << this->getY()
    << ", currentX = " << this->getX()    
    << ", goalDir(in degrees) =" << goalDir*180/M_PI
    << ", rotation angle passed to function: " << (goalDir)*180/M_PI);
    */
    this->rotateDegrees((goalDir)*180/M_PI);
 
}

double Steering::findRotatingAngle(float goalAngle)
{
    double goalDirDegree = (goalAngle * 180.00f) / M_PI;
    double r_currentDirDegree = (this->getTheta() * 180.00f) / M_PI;
    double rotatingAngleDegree = goalDirDegree - r_currentDirDegree;
	
    return rotatingAngleDegree;
}

//void Steering::PointAndShoot(const geometry_msgs::PoseStamped& msg)
void Steering::PointAndShoot()
{	
    //goalMessage(msg);
    pointToGoal();
    float deltaX = goal_xPos - this->getX();
    float deltaY = goal_yPos - this->getY();
    float distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    this->moveDist(distance, true);
    //Turn to the direction of the goal
    //TODO - Debug this one
    //double turnToGoalDirAngle = findRotatingAngle(goal_thetaPos);
    rotateDegrees(goal_thetaPos*180/M_PI);
    /*
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed
                    << "goal_x = " << goal_xPos
                    << ", goal_y = " << goal_yPos
                    << ", goal_direction = " << goal_thetaPos);
	*/
}

void Steering::Servoing()
{
	float deltaX = this->get_goal_Xpos() - this->getX();
    float deltaY = this->get_goal_Ypos() - this->getY();
	float rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
	float Atan = atan2(deltaY, deltaX);
	float alpha;
	float beta;
	float Kp = 1.0, Ka = 1.15, Kb = -0.05;
	float angle;
	
	alpha = -(this->getTheta()) + atan2(deltaY, deltaX);
	beta = -(this->getTheta()) - alpha;
		
	angle = ((Ka*alpha + Kb*beta))*180/M_PI;
	float dist = Kp*rho;
	
	ros::Rate rate(1000);
	
	//------------------------------------------------------------------------------//
	while(ros::ok())
	{
		deltaX = this->get_goal_Xpos() - this->getX();
		deltaY = this->get_goal_Ypos() - this->getY();
		rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
		
		alpha = -(this->getTheta()) + atan2(deltaY, deltaX);
		beta = -(this->getTheta()) - alpha;
				
		dist = Kp*rho;
		angle = ((Ka*alpha + Kb*beta))*180/M_PI;
		/*
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "x_pos = " << this->getX() 
		<< ", y_pos = " << this->getY() 
		<< ", deltaX " << deltaX 
		<< ", deltaY " << deltaY 
		<< ", rho = " << rho 
		<< ", alpha = " << alpha 
		<< ", beta = " << beta 
		<< ", theta_pos(in degrees) = " << this->getTheta()*180/M_PI 
		<< ", angle = " << angle 
		<< ", dist = " << dist);
		*/
		
		this->rotateDegrees(angle - this->getTheta()*180/M_PI);
		this->moveDist(dist, true);
		if(rho <= 0.15)
		{
			/*
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "goal achieved, will break from loop");
			*/
			this->stopMoving();
			break;
		}
		
		ros::spinOnce();
	}
	rotateDegrees(180);
}

void Steering::moveAndRotate(double movespeed, double rotationspeed)
{
	float currentTheta = this->getTheta();
	float currentX = this->getX();
	float currentY = this->getY();
		
	geometry_msgs::Twist vel_msg = getMsg();
	
	this->setMove(true);//sets moving attribute to true
		
	float speed_in_rad = angles::normalize_angle(angles::from_degrees(rotationspeed));	
	
	vel_msg.linear.x = movespeed;
	vel_msg.angular.z = speed_in_rad;
	
	(this->get_publisher()).publish(vel_msg);
}


void Steering::ServoingAlternative()
{
	float deltaX = this->get_goal_Xpos() - this->getX();
    float deltaY = this->get_goal_Ypos() - this->getY();
	float rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
	float Atan = atan2(deltaY, deltaX);
	float alpha;
	float beta;
	float Kp = 1.0, Ka = 1.15, Kb = -0.1;
	float angle;
	
	alpha = -(this->getTheta()) + atan2(deltaY, deltaX);
	beta = -(this->getTheta()) - alpha;
		
	angle = ((Ka*alpha + Kb*beta))*180/M_PI;
	float dist = Kp*rho;
	
	ros::Rate rate(1000);
	
	while(ros::ok())
	{
		deltaX = this->get_goal_Xpos() - this->getX();
		deltaY = this->get_goal_Ypos() - this->getY();
		rho = sqrt(pow(deltaX, 2) + pow(deltaY, 2));
		
		alpha = -(this->getTheta()) + atan2(deltaY, deltaX);
		beta = -(this->getTheta()) - alpha;
				
		dist = Kp*rho;
		angle = ((Ka*alpha + Kb*beta))*180/M_PI;
		/*
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "x_pos = " << this->getX() 
		<< ", y_pos = " << this->getY() 
		<< ", deltaX " << deltaX 
		<< ", deltaY " << deltaY 
		<< ", rho = " << rho 
		<< ", alpha = " << alpha 
		<< ", beta = " << beta 
		<< ", theta_pos(in degrees) = " << this->getTheta()*180/M_PI 
		<< ", angle = " << angle 
		<< ", dist = " << dist);
		*/
		this->moveAndRotate(dist, angle);
		/*
		ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "x_pos = " << this->getX() 
			<< ", y_pos = " << this->getY()  
			<< ", rho = " << rho);
		*/
		if(rho <= 0.15)
		{
			/*
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "x_pos = " << this->getX() 
			<< ", y_pos = " << this->getY() 
			<< ", deltaX " << deltaX 
			<< ", deltaY " << deltaY 
			<< ", rho = " << rho 
			<< ", alpha = " << alpha 
			<< ", beta = " << beta 
			<< ", theta_pos(in degrees) = " << this->getTheta()*180/M_PI 
			<< ", angle = " << angle 
			<< ", dist = " << dist);
			
			ROS_INFO_STREAM(std::setprecision(2) << std::fixed
			<< "goal achieved, will break from loop");
			*/
			this->stopMoving();
			break;
		}
		
		ros::spinOnce();
	}
	rotateDegrees(goal_thetaPos*180/M_PI);
	
}
