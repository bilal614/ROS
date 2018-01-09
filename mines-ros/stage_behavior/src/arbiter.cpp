#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

/**
 * Priority based arbiter, forwards highest priority commands
 */
class Arbiter
{
public:
	// members
	
	// methods
  Arbiter();
  void teleopCallback(const geometry_msgs::Twist &msg);
  void escape_behaviorCallback(const geometry_msgs::Twist &msg);
  void cruise_behaviorCallback(const geometry_msgs::Twist &msg);
  
  void publish(double angular, double linear);
  void setPriority();
  
  void arbitrate();

protected:
	
	enum Priority
    {
      TELE_OP,
      ESCAPE,
      CRUISE,
    };
    
	// members
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_;
	geometry_msgs::Twist cmd_vel_;
	Priority priority_;
	u_int8_t priority_mask_;
	// methods
};

///////////////////////////////////////////////////////////////////////////

Arbiter::Arbiter(): priority_mask_(0)
{
	ros::Subscriber sub = n.subscribe("cmd_vel0", 1, teleopCallback);
	ros::Subscriber sub = n.subscribe("cmd_vel1", 1, escape_behaviorCallback);
	ros::Subscriber sub = n.subscribe("cmd_vel2", 1, cruise_behaviorCallback);
	
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void Arbiter::teleopCallback(const geometry_msgs::Twist &msg)
{
	priority_mask_ |= 0x01;
}

void Arbiter::escape_behaviorCallback(const geometry_msgs::Twist &msg)
{
	priority_mask_ |= 0x02;	
}

void Arbiter::cruise_behaviorCallback(const geometry_msgs::Twist &msg)
{
	priority_mask_ |= 0x04;	
}

void Arbiter::setPriority()
{
	if(priority_mask_ & 0x01)
	{
		priority_ = TELE_OP;
		return;
	}
	else if(priority_mask_ & 0x02)
	{
		priority_ = ESCAPE;
		return;
	}
	else if(priority_mask_ & 0x04)
	{
		priority_ = CRUISE;
		return;
	}
	else
	{
		priority_mask_ = 0x00;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arbiter");
  Arbiter arbiter;
  ros::spin();

  return 0;
}
