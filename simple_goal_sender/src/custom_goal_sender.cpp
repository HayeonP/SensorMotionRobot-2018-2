#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;


  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action s erver to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  int n_data;
  std::vector<double> pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;

  if(!nh.getParam("/n_data", n_data)){
    ROS_ERROR("Cannot read n_data");
    return 1;
  }

  ROS_INFO("n_data : %d", n_data);

  if(!nh.getParam("/goal_pos_x", pos_x)){
    ROS_ERROR("Cannot read goal_pos_x");
    return 1;
  }

  if(!nh.getParam("/goal_pos_y", pos_y)){
    ROS_ERROR("Cannot read goal_pos_y");
    return 1;
  }
  
  if(!nh.getParam("/goal_pos_z", pos_z)){
    ROS_ERROR("Cannot read goal_pos_z");
    return 1;
  }

  if(!nh.getParam("/goal_ori_x", ori_x)){
    ROS_ERROR("Cannot read goal_ori_x");
    return 1;
  }

  if(!nh.getParam("/goal_ori_y", ori_y)){
    ROS_ERROR("Cannot read goal_ori_y");
    return 1;
  }

  if(!nh.getParam("/goal_ori_z", ori_z)){
    ROS_ERROR("Cannot read goal_ori_z");
    return 1;
  }

  if(!nh.getParam("/goal_ori_w", ori_w)){
    ROS_ERROR("Cannot read goal_ori_w");
    return 1;
  }


  for(int i = 0; i < n_data; i++){
    ROS_INFO("goal_pos_x : %lf", pos_x[i]);
    ROS_INFO("goal_pos_y : %lf", pos_y[i]);
    ROS_INFO("goal_pos_z : %lf", pos_z[i]);
    ROS_INFO("goal_ori_x : %lf", ori_x[i]);
    ROS_INFO("goal_ori_y : %lf", ori_y[i]);
    ROS_INFO("goal_ori_z : %lf", ori_z[i]);
    ROS_INFO("goal_ori_w : %lf", ori_w[i]);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();
  
    goal.target_pose.pose.position.x = pos_x[i];
    goal.target_pose.pose.position.y = pos_y[i];
    goal.target_pose.pose.position.z = pos_z[i];
    goal.target_pose.pose.orientation.x = ori_x[i];
    goal.target_pose.pose.orientation.y = ori_y[i];
    goal.target_pose.pose.orientation.z = ori_z[i];
    goal.target_pose.pose.orientation.w = ori_w[i];

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("success");
    else
      ROS_INFO("Fail");
  }
  
  

  
  return 0;

}
