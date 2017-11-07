/*This nodes generates a rectangular path and publishes on the /plan topic*/
/*We ares sure that the distance is equally divivded -> recalculation for look a head radius*/
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
		double weightPointDis);
double calculateDistance(point p1, point p2);
point calculateMidPoint(point p1, point p2);
bool checkforDistance(std::vector<point> points, double weightPointDis);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "global_planner_trig");

	ros::NodeHandle nh;

	ros::Publisher global_triangle_planner = nh.advertise<nav_msgs::Path> (
			"/plan", 1);

	while (ros::ok())
	{
		nav_msgs::Path msg;
		ROS_INFO("RUNNING RECTANGULAR GLOBAL PATH PLANNER");

		point p1, p2, p3;
		p1.x = 5;
		p1.y = 5;
		p2.x = 15;
		p2.y = 15;
		p3.x = 10;
		p3.y = 15;

		//msg = generateTrianglePath(p1, p2 , p3);
		msg = generateTrianglePath(p1, p2, p3, 2);

		global_triangle_planner.publish(msg);

		//ros::spinOnce();
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
 * @param weightPointDis - distance between 2 weightPoints
 * @return msg the constructed nav_msgs::Path message
 */
nav_msgs::Path generateTrianglePath(point p1, point p2, point p3,
		double weightPointDis)
{
	int Points_max = 3;
	ROS_INFO_STREAM(
			std::setprecision(2) << std::fixed << "pointMax: " << Points_max);

	double x_points[Points_max];
	double y_points[Points_max];

	//4 standard points
	double x_cors[3];
	double y_cors[3];
	x_cors[0] = p1.x;
	y_cors[0] = p1.y;

	x_cors[1] = p2.x;
	y_cors[1] = p2.y;

	x_cors[2] = p3.x;
	y_cors[2] = p3.y;

	//init first point,
	x_points[0] = p1.x;
	y_points[0] = p1.y;

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
	//Find the vecter of all points
	std::vector<point> points;
	points.push_back(p1);
	points.push_back(p2);
	ROS_INFO_STREAM(
			std::setprecision(2) << std::fixed << "points size init: "
					<< points.size());
	int count = 0;

	/*Testing
	 point midPoint = calculateMidPoint(points[0], points[1]);
	 points.insert(points.begin() + 1, midPoint);
	 bool checkForDis = checkforDistance(points, 10);
	 ROS_INFO_STREAM(
	 std::setprecision(2) << std::fixed << "checkForDis: " << checkForDis);*/

	//TODO - debug this
	/*while (!checkforDistance(points, weightPointDis)) // means still have distance bigger than weight point distance
	{
		for (int i = 1; i < points.size(); i++)
		{
			point midPoint = calculateMidPoint(points[i - 1], points[i]);

			ROS_INFO_STREAM(
					std::setprecision(2) << std::fixed <<"midPoint.x: " << midPoint.x
					<<", midPoint.y: " << midPoint.y);
			points.insert(points.begin() + i, midPoint);
		}
	}*/

	for (int i = 0; i < points.size(); i++)
	{
		ROS_INFO_STREAM(
				std::setprecision(2) << std::fixed << "point[" << i << "].x: "
						<< points[i].x << ", point[" << i << "].y: "
						<< points[i].y);
	}

	return Construct_Path_Msg(x_points, y_points, Points_max);
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
 * Used to check if all distances between two points is smaller than weight points distance
 * Return true if the does not contain any distance bigger than weight points distance
 */
bool checkforDistance(std::vector<point> points, double weightPointDis)
{
	bool returnVal = true;
	for (int i = 1; i < points.size(); i++)
	{
		if (calculateDistance(points[i - 1], points[i]) > weightPointDis)
		{
			returnVal = false;
			break;
		}
	}
	return returnVal;
}
nav_msgs::Path generateTrianglePath(point p1, point p2, point p3)
{
	double x_cors[3];
	double y_cors[3];

	//init fours point, in a clock-wise order
	x_cors[0] = p1.x;
	y_cors[0] = p1.y;

	x_cors[1] = p2.x;
	y_cors[1] = p2.y;

	x_cors[2] = p3.x;
	y_cors[2] = p3.y;

	return Construct_Path_Msg(x_cors, y_cors, 3);
}

