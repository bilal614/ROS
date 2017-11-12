/*This nodes generates a rectangular path and publishes on the /plan topic*/
/*The distance between 2 weight points should be calculated in the steering_listener*/
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <sstream>
struct point
{
	double x;
	double y;
};

//The distance between two way point should be = six as a constant

nav_msgs::Path Construct_Path_Msg(double* x, double *y, int nrOfPoints);
nav_msgs::Path generateRectangularPath(double w, double h, double wayPointDis,
		double x, double y);
nav_msgs::Path generateRectangularPath(double w, double h, double x, double y);
void getTriangleInput(double* w, double* h, double* distanceWeightPoints,
		double* startX, double* startY);
void GetParam(std::string paramName, double* value);
int main(int argc, char** argv)
{
	ros::init(argc, argv, "global_planner_reg");

	ros::NodeHandle nh;

	ros::Publisher global_retangle_planner = nh.advertise<nav_msgs::Path> (
			"/plan", 10);
	nav_msgs::Path msg;
	ROS_INFO("RUNNING RECTANGULAR GLOBAL PATH PLANNER");

	double h, w, wayPointsDis, x, y;
	GetParam("h", &h);
	GetParam("w", &w);
	GetParam("wayPointsDis", &wayPointsDis);
	GetParam("x", &x);
	GetParam("y", &y);
	while (ros::ok())
	{
		//msg = generateRectangularPath(9, 6, 3, 3, 3);
		//msg = generateRectangularPath(3, 3, 3, 2);
		msg = generateRectangularPath(w, h, wayPointsDis, x, y);
		global_retangle_planner.publish(msg);

		ros::spinOnce();
		//ros::spin();
	}

	return 0;
}

void GetParam(std::string paramName, double* value)
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
		ROS_INFO_STREAM(
				std::setprecision(2) << std::fixed << "\n << x: " << "[" << i
				<< "] = " << x[i] << "\n << y: " << "[" << i << "] = "
				<< y[i]);

	}

	msg.poses = poses;
	/*ROS_INFO_STREAM(
	 std::setprecision(2) << std::fixed << "\nPath Message: " << msg);*/
	return msg;
}
/**
 * generateRectangularPath function
 * Used to create a nav_msgs::Path in rectangular shape
 * @param w width of the rectangular
 * @param h height of the rectangular
 * @param x,y is the top left conner of the rectangular path, can be considered as a current position
 * @param wayPointDis - distance between 2 weightPoints
 * @return msg the constructed nav_msgs::Path message
 */
nav_msgs::Path generateRectangularPath(double w, double h, double wayPointDis,
		double x, double y)
{
	std::vector<point> points;

	double rightTopPointX = x + w;
	double tempX = x;
	point leftTopPoint;
	leftTopPoint.x = x;
	leftTopPoint.y = y;
	points.push_back(leftTopPoint);
	while (tempX < rightTopPointX)
	{
		point p;
		p.y = y;
		if (rightTopPointX - tempX >= wayPointDis)
			p.x = tempX + wayPointDis;
		else
			p.x = tempX + (rightTopPointX - tempX);

		points.push_back(p);
		tempX += wayPointDis;
	}

	double rightBottomY = y + h;
	double tempY = y;
	while (tempY < rightBottomY)
	{
		point p;
		p.x = rightTopPointX;
		if (rightBottomY - tempY >= wayPointDis)
			p.y = tempY + wayPointDis;
		else
			p.y = tempY + (rightBottomY - tempY);

		points.push_back(p);
		tempY += wayPointDis;
	}

	//temp x coordinates from right to left
	double tempX_RL = x + w;
	while (tempX_RL > x)
	{
		point p;
		p.y = rightBottomY;
		if(tempX_RL - x >= wayPointDis)
			p.x = tempX_RL - wayPointDis;
		else
			p.x = tempX_RL - (tempX_RL - x);
		points.push_back(p);
		tempX_RL -= wayPointDis;

	}

	//temp  y coordinates from bottom to top
	double tempY_BT = x + h;
	while (tempY_BT > y)
	{
		point p;
		p.y = y;
		if(tempY_BT - y >= wayPointDis)
			p.y = tempY_BT - wayPointDis;
		else
			p.y = tempY_BT - (tempY_BT - wayPointDis);
		points.push_back(p);
		tempY_BT -= wayPointDis;
	}

	double x_points[points.size()];
	double y_points[points.size()];

	for(int i = 0; i < points.size(); i++)
	{
		x_points[i] = points[i].x;
		y_points[i] = points[i].y;
	}

	return Construct_Path_Msg(x_points, y_points, points.size());
}

nav_msgs::Path generateRectangularPath(double w, double h, double x, double y)
{
	double x_cors[5];
	double y_cors[5];

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

	x_cors[4] = x;
	y_cors[4] = y;

	return Construct_Path_Msg(x_cors, y_cors, 5);
}

