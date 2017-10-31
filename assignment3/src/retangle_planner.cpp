/*This nodes generates a rectangular path and publishes on the /plan topic*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

nav_msgs::Path Construct_Path_Msg(double* x, double *y, int nrOfPoints);
nav_msgs::Path  generateRectangularPath(double w, double h, double x, double y);
int main(int argc, char** argv)
{
	ros::init(argc, argv, "global_planner_reg");

	ros::NodeHandle nh;

	ros::Publisher global_retangle_planner = nh.advertise<nav_msgs::Path> (
			"/path", 1000);

	ros::Rate rate(1000.0);
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		nav_msgs::Path msg;
		ROS_INFO("RUNNING RECTANGULAR GLOBAL PATH PLANNER");
		msg = generateRectangularPath(20,10,5,5);

		global_retangle_planner.publish(msg);

		ros::spin();
	}

	return 0;
}

/**
 * Construct_Path_Msg function
 * Used to populate a nav_msgs::Path given a list of x and y coordinates
 * @param x double* pointer to array containing x coordinates
 * @param y double* pointer to array containing y coordinates
 * @return msg the constructed nav_msgs::Path message
 */
nav_msgs::Path Construct_Path_Msg(double* x, double *y, int nrOfPoints)
{
	nav_msgs::Path msg;
	std::vector<geometry_msgs::PoseStamped> poses(nrOfPoints);
	for (int i = 0; i < nrOfPoints; i++)
	{
		poses.at(i).pose.position.x = x[i];
		poses.at(i).pose.position.y = y[i];
	}
	msg.poses = poses;
	return msg;
}
/**
 * generateRectangularPath function
 * Used to create a nav_msgs::Path in rectangular shape
 * @param w width of the rectangular
 * @param h height of the rectangular
 * @param x,y is the top left conner of the rectangular path, can be considered as a current position
 * @return msg the constructed nav_msgs::Path message
 */
nav_msgs::Path generateRectangularPath(double w, double h, double x, double y)
{
	double x_cors[4];
	double y_cors[4];

	//TODO this one can be improved in such a way that the nr of centers point can be added
	//init fours point, in a clock-wise order
	x_cors[0] = x;
	y_cors[0] = y;

	x_cors[1] = x + w;
	y_cors[1] = y;

	x_cors[2] = x + w;
	y_cors[2] = y + h;

	x_cors[3] = x;
	y_cors[3] = y + h;

	return Construct_Path_Msg(x_cors, y_cors, 4);
}
