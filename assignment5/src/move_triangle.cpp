#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>
#include <assignment5/Triangle.h>

void TriangleMessageCallback(const assignment5::Triangle& msg);
bool triangle_trigger = false;
assignment5::Triangle triangleDrawing;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_turtle");
	
	ros::NodeHandle nh;
	ros::Subscriber subs = nh.subscribe("cmd", 1000, &TriangleMessageCallback);

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<turtlebot_actions::TurtlebotMoveAction> ac(
			"turtlebot_move", true);
	
	while(!triangle_trigger)
	{
		ros::spinOnce();
	}
	
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	turtlebot_actions::TurtlebotMoveGoal goalAction;
	
	int count = 0;
	while(count < 3)
	{
		//goalAction.forward_distance = 4.0f;
		//goalAction.turn_distance = M_PI*2/3;
		goalAction.forward_distance = triangleDrawing.sideLength;
		if(count == 0)
		{
			goalAction.turn_distance = 0.0;
		}
		else
		{
			if(triangleDrawing.cw)
			{
				goalAction.turn_distance = M_PI*2/3;
			}
			else
			{
				goalAction.turn_distance = -1*M_PI*2/3;
			}
		}
		ac.sendGoal(goalAction);
		
		//actionlib::SimpleClientGoalState state = ac.sendGoalAndWait(goalAction, ros::Duration(30.0));
		//wait for the action to return
		bool finished_before_timeout = ac.waitForResult(ros::Duration(15.0));

		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else
			ROS_INFO("Action did not finish before the time out.");
			
		count++;
	}
	//exit
	return 0;
}

void TriangleMessageCallback(const assignment5::Triangle& msg)
{
	ROS_INFO_STREAM(std::setprecision(2) << std::fixed
		<< "Message: " << msg);
		
	triangleDrawing.sideLength = msg.sideLength;
	triangleDrawing.cw = msg.cw;
	triangle_trigger = true;
}
