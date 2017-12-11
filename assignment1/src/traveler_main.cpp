#include "traveler.hpp"

	

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "turtle_communicator");
	
	ros::NodeHandle nh;
	
	Sender sender(nh);
		
	ros::Subscriber subs = nh.subscribe("triangles", 1000, &Sender::triangleMessage, &sender);
	
	ros::spin();
	//sender.makeTriangle(4, true);
	/*
	while(ros::ok)
	{
	}	
	sender.makeTriangle(4, true);
	
	sender.makeTriangle(6, false);
	
	sender.makeTriangle(6, true);
	*/
}
