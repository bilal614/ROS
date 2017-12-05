#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_triangle");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> ac(
			"TurtlebotMoveAction", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	turtlebot_actions::TurtlebotMoveActionGoal goal;
	//TODO investiage the goal message of turtle_bot actions

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");

	//exit
	return 0;
}
