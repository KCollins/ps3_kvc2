#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ps3_kvc2/SineAction.h>

int main(int argc, char** argv) {
        ros::init(argc, argv, "sine_client"); // name this node 
        ps3_kvc2::SineGoal goal; 
        actionlib::SimpleActionClient<ps3_kvc2::SineAction> action_client("sine", true);
        
        // attempt to connect to the server:
        ROS_INFO("waiting for server");
        bool server_exists = action_client.waitForServer(ros::Duration(15.0)); // wait for up to 5 seconds
        // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
        //bool server_exists = action_client.waitForServer(); //wait forever

        if (!server_exists) {
            ROS_WARN("could not connect to server; halting");
            return 0; // bail out; optionally, could print a warning message and retry
        }
        
       
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        while(true) {
        // stuff a goal message:
            ps3_kvc2::SineGoal goal;
            std_msgs::Float64 amplitude;
            std_msgs::Float64 frequency;
            std_msgs::Int32 cycles;
            std::cout << "Please enter your desired amplitude. \n";
            std::cin >> amplitude.data;
            goal.amplitude = amplitude.data;
            std::cout << "Please enter your desired frequency, in rads/sec. \n";
            std::cin >> goal.frequency;
            std::cout << "Please enter a desired number of cycles. \n";
            std::cin >> goal.cycles;
        
        action_client.sendGoal(goal); // simple example--send goal, but do not specify callbacks
        //action_client.sendGoal(goal,&doneCb); 

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result");
            return 0;
        }
        else {
          //if here, then server returned a result to us
        }
        
      }

    return 0;
}
