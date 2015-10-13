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
  //std::string action_name_;
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


SineServer::SineServer() :
   as_(nh_, "sine", boost::bind(&SineServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "sine"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of SineServer...");

    as_.start(); //start the server running
    ROS_INFO("Server has started for realsies.");
}


void SineServer::executeCB(const actionlib::SimpleActionServer<ps3_kvc2::SineAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");

    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
    
    //ROS_INFO("%s: Executing, creating a sine wave of %i cycles with amplitude %f and frequency %f.", action_name_.c_str(), goal->cycles, goal->amplitude, goal->frequency);
    
    amp.data = goal->amplitude;
    freq.data = goal->frequency;
    cyc.data = goal->cycles;

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "Sine");
  ROS_INFO("Server main has started.");
    ros::NodeHandle n;
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);

    SineServer foo; //Create an instance of the server
ROS_INFO("Server instance created.");
  input_float.data = 0.0;
  vel_cmd.data = 0.0; 
  ros::Rate naptime(1/dt); //create a ros object from the ros “Rate” class;

while (ros::ok())
    {
        ros::spinOnce();
        period.data=(2 * PI)/freq.data; //duration of one cycle
        duration=period.data*cyc.data; //total duration, in seconds.

      if (cycle_count <= cyc.data){
        vel_cmd.data=amp.data*sin(2 * PI * freq.data * input_float.data);
        ROS_INFO("Generating a sine wave now...");
        input_float.data = input_float.data + dt; // increment by dt each iteration
        cycle_count=cycle_count+(dt/period.data);

      } else {
      	ROS_INFO("Requested number of cycles ends");
        input_float.data = 0.0;
        vel_cmd.data = 0.0;
        cycle_count = 0.0;
      }

        my_publisher_object.publish(vel_cmd); // publish the value
  naptime.sleep(); 

    
}}