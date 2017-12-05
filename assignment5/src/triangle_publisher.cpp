#include "ros/ros.h"
#include "assignment5/Triangle.h"
#include <sstream>
#include <iostream>

using namespace std;

void GetFloatParam(std::string paramName, float* value);
void GetBoolParam(std::string paramName, bool* value);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publishTriangleInfo");

	ros::NodeHandle n;


	ros::Publisher triangle_pub = n.advertise<assignment5::Triangle>("cmd",10);


	ros::Rate loop_rate(10);


	while (ros::ok())
	{
		//Message object
		assignment5::Triangle msg;

		ROS_INFO("STARTING PUBLISHER FOR SENDING TRIANGLE INFO");

		//Get message from parametter server
		bool cw;
		GetBoolParam("cw", &cw);
		msg.cw = cw;
		GetFloatParam("sideLength", &msg.sideLength);


		ROS_INFO("%f %d", msg.sideLength, msg.cw);

		triangle_pub.publish(msg);

		ros::spin();
	}

  return 0;
}

void GetFloatParam(std::string paramName, float* value)
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

void GetBoolParam(std::string paramName, bool* value)
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
