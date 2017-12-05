#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <assignment5/Triangle.h>

/*Glocal variables*/
ros::Subscriber cmd_sub;
//Received triangle message
assignment5::Triangle re_msg;

/*Print the received messages from publisher*/
void cmdCallback(const assignment5::Triangle::ConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_triangle");

	//Handler
	ros::NodeHandle n;

	cmd_sub = n.subscribe("cmd", 10, cmdCallback);

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

//TODO - Debug this
void cmdCallback(const assignment5::Triangle::ConstPtr& msg) {
	re_msg.sideLength = msg->sideLength;
	re_msg.cw = msg->cw;
	ROS_INFO("Triangle info: sideLength: [%f] - cw: [%d]", msg->sideLength, msg->cw);
	/*Draw triangle with red color*/
}
