#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <action_basic/fibonacciAction.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "action_basic_client");
  actionlib::SimpleActionClient<action_basic::fibonacciAction> ac("fibonacci",true);

  ROS_INFO("Wait for server");
  ac.waitForServer();
  ROS_INFO("Server is connected");

  action_basic::fibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  bool finished = ac.waitForResult(ros::Duration(30.0));
  if(finished){
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("%s", state.toString().c_str());
  }
  else{
    ROS_INFO("Timeout");
  }

  return 0;
}




// #include <ros/ros.h>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>
// #include <action_basic/fibonacciAction.h>

// int main(int argc, char** argv){
//   ros::init(argc, argv, "action_basic_client");
//   actionlib::SimpleActionClient<action_basic::fibonacciAction> ac("fibonacci", true);
  
//   ROS_INFO("Wait for server");
//   ac.waitForServer();
//   ROS_INFO("Server is connected!");

//   action_basic::fibonacciGoal goal;
//   goal.order = 20;
//   ac.sendGoal(goal);

//   bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

//   if(finished_before_timeout){
//     actionlib::SimpleClientGoalState state = ac.getState();
//     ROS_INFO("%s", state.toString().c_str());
//   }
//   else{
//     ROS_INFO("Timeout!");
//   }
  
//   return 0;
// }