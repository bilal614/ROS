/*This nodes generates a rectangular path and publishes on the /plan topic*/
/*The distance between 2 weight points should be calculated in the steering_listener*/
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <sstream>

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
		msg = generateRectangularPath(h,w,wayPointsDis,x,y);
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
	int heightWeightPoints = h / wayPointDis;
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "heightWeightPoints: " << heightWeightPoints);
	int widthWeightPoints = w / wayPointDis + 1;
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "widthWeightPoints: " << widthWeightPoints);
	int countPoints = 1;
	double remainderWidth = remainder(w, wayPointDis);
	double remainderHeight = remainder(h, wayPointDis);
	if (remainderWidth > 0)
		widthWeightPoints++;
	if (remainderHeight > 0)
		heightWeightPoints++;
	/*ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "widthWeightPoints: " << widthWeightPoints
	 << ", remainderWidth: " << remainderWidth);*/
	/*if(remainderHeight > 0)
	 heightWeightPoints ++;*/ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "heightWeightPoints: " << heightWeightPoints
			<< ", remainderHeight: " << remainderHeight);
	int Points_max;
	Points_max = heightWeightPoints * 2 + widthWeightPoints * 2; // minus the starting point
	//TODO - Debug this
	//ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "pointMax: " << Points_max);
	double x_points[Points_max];
	double y_points[Points_max];

	//4 standard points
	double x_cors[4];
	double y_cors[4];
	x_cors[0] = x;
	y_cors[0] = y;

	x_cors[1] = x + w;
	y_cors[1] = y;

	x_cors[2] = x + w;
	y_cors[2] = y + h;

	x_cors[3] = x;
	y_cors[3] = y + h;

	//TODO this one can be improved in such a way that the nr of centers point can be added
	//init first point,
	x_points[0] = x;
	y_points[0] = y;

	for (int i = 1; i < widthWeightPoints; i++)
	{
		y_points[countPoints] = y_cors[0];
		double tempDis = x_cors[1] - x_points[countPoints - 1];
		/*ROS_INFO_STREAM(
		 std::setprecision(2) << std::fixed << "\ntempDis: " << tempDis
		 << "\nx_point(count - 1): "
		 << x_points[countPoints - 1]);*/
		if (tempDis < wayPointDis)
		{
			x_points[countPoints] = x_points[countPoints - 1] + tempDis;
			/*ROS_INFO_STREAM(
			 std::setprecision(2) << std::fixed << "\ncountPoint: "
			 << countPoints << "\nx_point(count): "
			 << x_points[countPoints]);*/

		}
		else
		{
			x_points[countPoints] = x_points[countPoints - 1] + wayPointDis;
		}

		countPoints++;
	}

	for (int i = 0; i < heightWeightPoints; i++)
	{

		x_points[countPoints] = x_cors[1];

		double tempDis = y_cors[2] - y_points[countPoints - 1];

		if (tempDis < wayPointDis)
		{
			//In case of the last point and remainder is diferent from weightPoint

			y_points[countPoints] = y_points[countPoints - 1] + tempDis;

		}
		else
		{
			y_points[countPoints] = y_points[countPoints - 1] + wayPointDis;
		}
		countPoints++;
	}
	//x_points[countPoints - 1] = x_cors[2];
	//y_points[countPoints - 1] = y_cors[2];

	for (int i = 0; i < widthWeightPoints; i++)
	{
		y_points[countPoints] = y_cors[2];

		double tempDis = x_points[countPoints - 1] - x_cors[3];
		if (tempDis < wayPointDis)
		{
			//In case of the last point and remainder is diferent from weightPoint

			x_points[countPoints] = x_points[countPoints - 1] - tempDis;

		}
		else
		{
			x_points[countPoints] = x_points[countPoints - 1] - wayPointDis;
		}

		countPoints++;

	}

	for (int i = 0; i < heightWeightPoints; i++)
	{

		x_points[countPoints] = x_cors[0];
		double tempDis = y_points[countPoints - 1] - y_cors[0];
		if (tempDis < wayPointDis)
		{
			//In case of the last point and remainder is diferent from weightPoint
			y_points[countPoints] = y_points[countPoints - 1] - tempDis;

		}
		else
		{
			y_points[countPoints] = y_points[countPoints - 1] - wayPointDis;
		}

		y_points[countPoints] = y_points[countPoints - 1] - wayPointDis;
		countPoints++;
	}

	if (remainderHeight != 0 && remainderWidth != 0)
	{
		x_points[countPoints] = x_cors[0];
		y_points[countPoints] = y_cors[0];
	}

	return Construct_Path_Msg(x_points, y_points, Points_max);
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

