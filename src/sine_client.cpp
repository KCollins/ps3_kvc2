#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ps3_kvc2/SineAction.h>

using namespace std;
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_Sine");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ps3_kvc2::SineAction> ac("Sine", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started!");

  ps3_kvc2::SineGoal goal;
  std::cout << "Hello, world! \n";
  std::cout << "Please enter your desired amplitude. \n";
  std::cin >> goal.amplitude;
    std::cout << "Please enter your desired frequency, in rads/sec. \n";
  std::cin >> goal.frequency;
  std::cout << "Please enter a desired number of cycles. \n";
  std::cin >> goal.cycles;

  // send a goal to the action
  ROS_INFO("Sending goal.");
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
