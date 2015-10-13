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
//std_msgs::Float64 dt;

class SineAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<ps3_kvc2::SineAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  ps3_kvc2::SineGoal goal_;
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

    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating Sine wave of %i with seeds %i, %i", action_name_.c_str(), goal->cycles, feedback_.sequence[0], feedback_.sequence[1]);
    ROS_INFO("%s: Executing, creating a sine wave of %i cycles with amplitude %f and frequency %f.", action_name_.c_str(), goal->cycles, goal->amplitude, goal->frequency);
    
    amp.data = goal->amplitude;
    freq.data = goal->frequency;
    cyc.data = goal->cycles;
  }
};

using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "Sine");
  ROS_INFO("Server is ready.");
    ros::NodeHandle n;
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);
  SineAction Sine(ros::this_node::getName());
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
  //ros::spinOnce();
  //ros::spin();
  return 0;
    }
}