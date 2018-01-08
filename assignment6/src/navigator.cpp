#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void GetFloatParam(std::string paramName, float* value);
const int NrOfCoordinates = 4;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_navigation_goals");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	int counter = 0;
	while(counter < NrOfCoordinates)
	{
		goal.target_pose.header.frame_id = "base_link";
		goal.target_pose.header.stamp = ros::Time::now();
		float x[NrOfCoordinates] = {0.0};
		float y[NrOfCoordinates] = {0.0};
		float w[NrOfCoordinates] = {1.0};
		switch(counter)
		{
			case 0:
				GetFloatParam("poseX1", &x[counter]); GetFloatParam("poseY1", &y[counter]); break;
			case 1:
				GetFloatParam("poseX2", &x[counter]); GetFloatParam("poseY2", &y[counter]);	break;
			case 2:
				GetFloatParam("poseX3", &x[counter]); GetFloatParam("poseY3", &y[counter]);	break;
			case 3:
				GetFloatParam("poseX4", &x[counter]); GetFloatParam("poseY4", &y[counter]);	break;
			default:
				break;
		}
		goal.target_pose.pose.position.x = x[counter];
		goal.target_pose.pose.position.y = y[counter];
		goal.target_pose.pose.orientation.w = w[counter];

		ROS_INFO("Sending goal");
		ac.sendGoal(goal);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base moved to your goal.");
		else
			ROS_INFO("The base failed to move to your goal for some reason");
		counter++;
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
