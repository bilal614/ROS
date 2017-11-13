/*This nodes generates a rectangular path and publishes on the /plan topic*/
/*We ares sure that the distance is equally divided -> recalculation for look a head radius*/
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>

struct point
{
	double x;
	double y;
};


nav_msgs::Path Construct_Path_Msg(double* x, double *y, int nrOfPoints);
nav_msgs::Path generateTrianglePath(point p1, point p2, point p3);
nav_msgs::Path generateTrianglePath(point p1, point p2, point p3,
		double wayPointDis);
double calculateDistance(point p1, point p2);
point calculateMidPoint(point p1, point p2);
bool checkforDistance(std::vector<point> points, double wayPointDis);
std::vector<point> generatedPoints(point p1, point p2, double wayPointDis);
void GetParam(std::string paramName, double* value);
int main(int argc, char** argv)
{
	ros::init(argc, argv, "global_planner_trig");

	ros::NodeHandle nh;

	ros::Publisher global_triangle_planner = nh.advertise<nav_msgs::Path> (
			"/plan", 10);

	point p1, p2, p3;
	double wayPointsDis = 0.0;
	GetParam("x1", &p1.x);
	GetParam("y1", &p1.y);
	GetParam("x2", &p2.x);
	GetParam("y2", &p2.y);
	GetParam("x3", &p3.x);
	GetParam("y3", &p3.y);
	GetParam("wayPointsDis", &wayPointsDis);

	while (ros::ok())
	{
		nav_msgs::Path msg;
		ROS_INFO("RUNNING RECTANGULAR GLOBAL PATH PLANNER");

		//msg = generateTrianglePath(p1, p2 , p3);
		msg = generateTrianglePath(p1, p2, p3, wayPointsDis);

		global_triangle_planner.publish(msg);

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
 * generateTrianglePath function
 * Used to create a nav_msgs::Path in rectangular shape
 * @param wayPointDis - distance between 2 weight points of the path
 * @return msg the constructed nav_msgs::Path message
 */
nav_msgs::Path generateTrianglePath(point p1, point p2, point p3,
		double wayPointDis)
{
	//For the first edge (p1,p2)
	double dis_p12 = calculateDistance(p1, p2);
	ROS_INFO_STREAM(
			std::setprecision(2) << std::fixed << "dis_p12: " << dis_p12);
	//For the second edge (p2,p3)
	double dis_p23 = calculateDistance(p2, p3);
	ROS_INFO_STREAM(
			std::setprecision(2) << std::fixed << "dis_p32: " << dis_p23);
	//For the first edge (p3,p1)
	double dis_p31 = calculateDistance(p3, p1);
	ROS_INFO_STREAM(
			std::setprecision(2) << std::fixed << "dis_p31: " << dis_p31);

	std::vector<point> points_p1_p2 = generatedPoints(p1, p2, wayPointDis);
	std::vector<point> points_p2_p3 = generatedPoints(p2, p3, wayPointDis);
	std::vector<point> points_p3_p1 = generatedPoints(p3, p1, wayPointDis);
	int Points_max = points_p1_p2.size() + points_p2_p3.size()
			+ points_p3_p1.size() - 2;
	ROS_INFO_STREAM(
			std::setprecision(2) << std::fixed << "pointMax: " << Points_max);
	double x_points[Points_max];
	double y_points[Points_max];
	int count = 0;

	for (int i = 0; i < points_p1_p2.size(); i++)
	{
		x_points[count] = points_p1_p2[i].x;
		y_points[count] = points_p1_p2[i].y;
		count++;
	}

	for (int i = 1; i < points_p2_p3.size(); i++)
	{
		x_points[count] = points_p2_p3[i].x;
		y_points[count] = points_p2_p3[i].y;
		count++;
	}

	for (int i = 1; i < points_p3_p1.size(); i++)
	{
		x_points[count] = points_p3_p1[i].x;
		y_points[count] = points_p3_p1[i].y;
		count++;
	}

	return Construct_Path_Msg(x_points, y_points, Points_max);
}

std::vector<point> generatedPoints(point p1, point p2, double wayPointDis)
{
	//Find the vecter of all points
	std::vector<point> points_between_p1_p2;
	points_between_p1_p2.push_back(p1);
	points_between_p1_p2.push_back(p2);
	/*ROS_INFO_STREAM(
			std::setprecision(2) << std::fixed << "points size init: "
			<< points_between_p1_p2.size());*/
	int count = 0;
	while (!checkforDistance(points_between_p1_p2, wayPointDis)) // means still have distance bigger than weight point distance
	{
		int point_size = points_between_p1_p2.size();
		for (int i = 1; i < point_size; i++)
		{
			if (calculateDistance(points_between_p1_p2[i - 1],
					points_between_p1_p2[i]) > wayPointDis)
			{
				point midPoint = calculateMidPoint(points_between_p1_p2[i - 1],
						points_between_p1_p2[i]);

				/*ROS_INFO_STREAM(
						std::setprecision(2) << std::fixed <<"midPoint.x: " << midPoint.x
						<<", midPoint.y: " << midPoint.y);*/

				points_between_p1_p2.insert(points_between_p1_p2.begin() + i,
						midPoint);
			}
		}
	}

	for (int i = 0; i < points_between_p1_p2.size(); i++)
	{
		ROS_INFO_STREAM(
				std::setprecision(2) << std::fixed << "point[" << i << "].x: "
				<< points_between_p1_p2[i].x << ", point[" << i << "].y: "
				<< points_between_p1_p2[i].y);
	}

	return points_between_p1_p2;
}
double calculateDistance(point p1, point p2)
{
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}
point calculateMidPoint(point p1, point p2)
{
	point temp;
	temp.x = (p1.x + p2.x) / 2.0;
	temp.y = (p1.y + p2.y) / 2.0;
	return temp;
}
/**
 * check for distance function
 * Used to check if all distances between two points is smaller than wayPointDis
 * Return true if the does not contain any distance bigger than weight points distance
 */
bool checkforDistance(std::vector<point> points, double wayPointDis)
{
	bool returnVal = true;
	for (int i = 1; i < points.size(); i++)
	{
		if (calculateDistance(points[i - 1], points[i]) > wayPointDis)
		{
			returnVal = false;
			break;
		}
	}
	return returnVal;
}
nav_msgs::Path generateTrianglePath(point p1, point p2, point p3)
{
	double x_cors[4];
	double y_cors[4];

	//init fours point, in a clock-wise order
	x_cors[0] = p1.x;
	y_cors[0] = p1.y;

	x_cors[1] = p2.x;
	y_cors[1] = p2.y;

	x_cors[2] = p3.x;
	y_cors[2] = p3.y;

	x_cors[3] = p1.x;
	y_cors[3] = p1.y;

	return Construct_Path_Msg(x_cors, y_cors, 3);
}


