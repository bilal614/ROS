#include "steering.hpp"
#include <sstream>

using namespace std;

void printLocationInfo(Steering &st);
int getMode();

int main(int argc, char **argv)
{
	
	//////////////////////////////////////////////
	
	ros::init(argc, argv, "steering_communicator");
     
    ros::NodeHandle nh;
     
    Steering steering(nh);
     
	//steering.initSteering();
    int mode;
    geometry_msgs::PoseStamped pose_msg;
	/*
	pose_msg.pose.orientation.x = M_PI_2;
	pose_msg.pose.orientation.y = M_PI_2;
	pose_msg.pose.orientation.z = M_PI_2;
	pose_msg.pose.orientation.w = M_PI_2;
	
	
	pose_msg.pose.position.x = 0;
	pose_msg.pose.position.y = 0;
	*/
	//steering.goalMessage(pose_msg);
	//steering.PointAndShoot();
	//steering.ServoingAlternative();
	
    while(ros::ok)
    {
		mode = getMode();
		
		if(mode == 0)
		{
			ROS_INFO("IN POINT AND SHOOT MODE: ");
			steering.PointAndShoot();
			printLocationInfo(steering);
		}
		else
		{
			ROS_INFO("IN SERVORING MODE: ");
			steering.ServoingAlternative();
			printLocationInfo(steering);
		}
	}
    
	//////////////////////////

    ros::spin();
     
    return 0;	
}


void printLocationInfo(Steering &st)
{
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "x_pos = " << st.getX()
		<< ", y_pos = " << st.getY()
		<< ", theta_pos = " << st.getTheta()
		<< ", theta_pos(in degrees) = " << st.getTheta()*180/M_PI);
}

int getMode()
{
    ROS_INFO("enter '0' for 'point and shoot' and '1' for servoring: ");
    //Make sure the string is correct
    string inputString;
    std::getline(std::cin, inputString);
 
    if(inputString == "0")
    {
        return 0;
    }
    else
    {
        return 1;
    }
}
	
