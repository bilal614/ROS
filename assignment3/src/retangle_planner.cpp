/*This nodes generates a rectangular path and publishes on the /plan topic*/
/*We ares sure that the distance is equally divivded -> recalculation for look a head radius*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <cmath>

nav_msgs::Path Construct_Path_Msg(double* x, double *y, int nrOfPoints);
nav_msgs::Path generateRectangularPath(double w, double h,
		double weightPointDis, double x, double y);
int main(int argc, char** argv)
{
	ros::init(argc, argv, "global_planner_reg");

	ros::NodeHandle nh;

	ros::Publisher global_retangle_planner = nh.advertise<nav_msgs::Path> (
			"/plan", 1000);

	ros::Rate rate(1000.0);
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		nav_msgs::Path msg;
		ROS_INFO("RUNNING RECTANGULAR GLOBAL PATH PLANNER");
		msg = generateRectangularPath(22, 12, 5, 5, 5);

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
		ROS_INFO_STREAM(
				std::setprecision(2) << std::fixed << "\n << x: " << "[" << i
						<< "] = " << x[i] << "\n << y: " << "[" << i << "] = "
						<< y[i]);

	}
	ROS_INFO_STREAM(
			std::setprecision(2) << std::fixed << "\nPath Message: " << msg);
	msg.poses = poses;
	return msg;
}
/**
 * generateRectangularPath function
 * Used to create a nav_msgs::Path in rectangular shape
 * @param w width of the rectangular
 * @param h height of the rectangular
 * @param x,y is the top left conner of the rectangular path, can be considered as a current position
 * @param weightPointDis - distance between 2 weightPoints
 * @return msg the constructed nav_msgs::Path message
 */
nav_msgs::Path generateRectangularPath(double w, double h,
		double weightPointDis, double x, double y)
{
	int heightWeightPoints = (h - y) / weightPointDis;
	int widthWeightPoints = (w - x) / weightPointDis;
	int countPoints = 1;

	int Points_max = heightWeightPoints * 2 + widthWeightPoints * 2 + 8;
	double x_points[Points_max];
	double y_points[Points_max];
	double remainderWidth = remainder(w, weightPointDis);
	double remainderHeight = remainder(h, weightPointDis);
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

	for (int i = 0; i <= widthWeightPoints + 1; i++)
	{

		double tempDis = x_cors[1] - x_points[countPoints - 1];
		ROS_INFO_STREAM(
							std::setprecision(2) << std::fixed << "\ntempDis: " << tempDis
							<< "\nx_point(count - 1): " << x_points[countPoints - 1]);
		if (tempDis < weightPointDis)
		{
			x_points[countPoints] = x_points[countPoints - 1] + tempDis;
			ROS_INFO_STREAM(
					std::setprecision(2) << std::fixed << "\ncountPoint: " << countPoints
					<< "\nx_point(count): " << x_points[countPoints]);

		}
		else
		{
			x_points[countPoints] = x_points[countPoints - 1] + weightPointDis;
		}
		y_points[countPoints] = y_cors[0];

		countPoints++;

	}


	for (int i = 0; i <= heightWeightPoints + 1; i++)
	{

		x_points[countPoints] = x_cors[1];

		double tempDis = y_cors[2] - y_points[countPoints - 1];

		if (tempDis < weightPointDis)
		{
			//In case of the last point and remainder is diferent from weightPoint

			y_points[countPoints] = y_points[countPoints - 1] + tempDis;

		}
		else
		{
			y_points[countPoints] = y_points[countPoints - 1] + weightPointDis;
		}
		countPoints++;
	}
	//x_points[countPoints - 1] = x_cors[2];
	//y_points[countPoints - 1] = y_cors[2];

	for (int i = 0; i <= widthWeightPoints; i++)
	{

		double tempDis = x_points[countPoints - 1] - x_cors[3];
		if (tempDis < weightPointDis)
		{
			//In case of the last point and remainder is diferent from weightPoint

			x_points[countPoints] = x_points[countPoints - 1] - tempDis;

		}
		else
		{
			x_points[countPoints] = x_points[countPoints - 1] - weightPointDis;
		}
		y_points[countPoints] = y_cors[2];
		countPoints++;

	}
	//x_points[countPoints] = x_cors[3];
	//y_points[countPoints] = y_cors[3];

	for (int i = 0; i < heightWeightPoints; i++)
	{

		double tempDis = y_points[countPoints - 1] - y_cors[0];
		if (tempDis < weightPointDis)
		{
			//In case of the last point and remainder is diferent from weightPoint
			y_points[countPoints] = y_points[countPoints - 1] - tempDis;

		}
		else
		{
			y_points[countPoints] = y_points[countPoints - 1] - weightPointDis;
		}
		x_points[countPoints] = x_cors[3];
		y_points[countPoints] = y_points[countPoints - 1] - weightPointDis;
		countPoints++;
	}

	return Construct_Path_Msg(x_points, y_points, Points_max);
}
