#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_actions/TurtlebotMoveAction.h>

/*This class have not worked yet, it contains some errors*/
/*class TurtlebotMoveAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_actions::TurtlebotMoveAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  turtlebot_actions::TurtlebotMoveActionFeedback feedback_;
  turtlebot_actions::TurtlebotMoveActionResult result_;

public:

  TurtlebotMoveAction(std::string name):
    as_(nh_, name, boost::bind(&TurtlebotMoveAction::executeCB, this, _1), false), action_name_(name)
  {
    as_.start();
  }

  ~TurtlebotMoveAction(void)
  {
  }

  void executeCB(const turtlebot_actions::TurtlebotMoveActionConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // push_back the seeds for the testing sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating message sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }

};*/


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Turtlebot_move_action_server");

  ROS_INFO("Runing action_server");

  //TurtlebotMoveAction turtlebotMoveAction("Turtlebot_move_action_server");

  ros::spin();

  return 0;
}
