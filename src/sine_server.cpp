#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <ps3_kvc2/SineAction.h>

class SineAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<ps3_kvc2::SineAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  ps3_kvc2::SineFeedback feedback_;
  ps3_kvc2::SineResult result_;

public:

  SineAction(std::string name) :
    as_(nh_, name, boost::bind(&SineAction::executeCB, this, _1), false),
    action_name_(name)
  {
     ROS_INFO("Class construction.");
    as_.start();
  }

  ~SineAction(void)
  {
  }

  void executeCB(const ps3_kvc2::SineGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    ROS_INFO("Executing executeCB");

    // push_back the seeds for the Sine sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating Sine wave of %i with seeds %i, %i", action_name_.c_str(), goal->cycles, feedback_.sequence[0], feedback_.sequence[1]);
    ROS_INFO("%s: Executing, creating Sine wave of %i cycles with amplitude %f and frequency %f.", action_name_.c_str(), goal->cycles, goal->amplitude, goal->frequency);

    // start executing the action
    for(int i=1; i<=goal->cycles; i++)
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


};

using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Sine");
  ROS_INFO("Server is ready.");
  SineAction Sine(ros::this_node::getName());
  ros::spin();

  return 0;
}