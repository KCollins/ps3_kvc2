#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <ps3_kvc2/SineAction.h>

#define PI 3.14159265
#define dt .010

std_msgs::Float64 amp;
std_msgs::Float64 freq;
std_msgs::Int32 cyc;
double cycle_count;
std_msgs::Float64 period;
double duration;
std_msgs::Float64 input_float, vel_cmd; 

class SineServer {
private:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<ps3_kvc2::SineAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  ps3_kvc2::SineGoal goal_;
  ps3_kvc2::SineFeedback feedback_;
  ps3_kvc2::SineResult result_;


public:
    SineServer(); //define the body of the constructor outside of class definition

    ~SineServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<ps3_kvc2::SineAction>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class SineServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

SineServer::SineServer() :
   as_(nh_, "sine", boost::bind(&SineServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "sine"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of SineServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}


void SineServer::executeCB(const actionlib::SimpleActionServer<ps3_kvc2::SineAction>::GoalConstPtr& goal) {
    //ROS_INFO("in executeCB");
    //ROS_INFO("goal input is: %d", goal->input);
    //do work here: this is where your interesting code goes
    
    //....

    // for illustration, populate the "result" message with two numbers:
    // the "input" is the message count, copied from goal->input (as sent by the client)
    // the "goal_stamp" is the server's count of how many goals it has serviced so far
    // if there is only one client, and if it is never restarted, then these two numbers SHOULD be identical...
    // unless some communication got dropped, indicating an error
    // send the result message back with the status of "success"

    ros::Rate r(1);
    bool success = true;
    ROS_INFO("Executing executeCB");

    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating Sine wave of %i with seeds %i, %i", action_name_.c_str(), goal->cycles, feedback_.sequence[0], feedback_.sequence[1]);
    ROS_INFO("%s: Executing, creating a sine wave of %i cycles with amplitude %f and frequency %f.", action_name_.c_str(), goal->cycles, goal->amplitude, goal->frequency);
    
    amp.data = goal->amplitude;
    freq.data = goal->frequency;
    cyc.data = goal->cycles;
    
    // the class owns the action server, so we can use its member methods here
   
    // DEBUG: if client and server remain in sync, all is well--else whine and complain and quit
    // NOTE: this is NOT generically useful code; server should be happy to accept new clients at any time, and
    // no client should need to know how many goals the server has serviced to date

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "Sine");
  ROS_INFO("Server is ready.");
    ros::NodeHandle n;
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);
  input_float.data = 0.0;
  vel_cmd.data = 0.0; 
  ros::Rate naptime(1/dt); //create a ros object from the ros “Rate” class;
   //set the sleep timer for 100Hz repetition rate (arg is in units of Hz)

while (ros::ok())
    {
        ros::spinOnce();
        period.data=(2 * PI)/freq.data; //duration of one cycle
        duration=period.data*cyc.data; //total duration, in seconds.

      if (cycle_count >= duration){
        vel_cmd.data=amp.data*sin(2 * PI * freq.data * input_float.data);
        ROS_INFO("Generating a sine wave now...");
        input_float.data = input_float.data + dt; // increment by dt each iteration
        cycle_count=cycle_count+(dt/period.data);

      } else {
        input_float.data = 0.0;
        vel_cmd.data = 0.0;
        cycle_count = 0.0;
      }

        my_publisher_object.publish(vel_cmd); // publish the value--of type Float64--
        // to the topic "topic1"
  // the next line will cause the loop to sleep for the balance of the desired period 
        // to achieve the specified loop frequency 
  naptime.sleep(); 

    return 0;
}}