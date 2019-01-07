#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <goal_point_collector/name.h>

#define MAX_PATH 512

class GoalPointSaver{
public:
  GoalPointSaver(char* _filename){
    client_ = nh_.serviceClient<goal_point_collector::name>("goal_save");
    getcwd(fileBuffer, MAX_PATH);
    filename = std::string(fileBuffer);
    filename = filename + "/" + std::string(_filename);

    ROS_INFO("Filename : %s", filename.c_str());

    srv.request.filename = filename;

    if(client_.call(srv)){
      ROS_INFO("save complete!");
    }
    else{
      ROS_INFO("goal_point_saver has faied due to some reasons");
    }

    return;
  }
private:
  ros::NodeHandle nh_;
  std::string filename;
  ros::ServiceClient client_;
  goal_point_collector::name srv;
  char fileBuffer[MAX_PATH];
};

int main(int argc, char** argv){
  
  if(argc !=2){
    ROS_ERROR("Please enter filename");

    return 1;
  }

  ros::init(argc, argv, "goal_point_saver");

  GoalPointSaver goalPointSaver(argv[1]);


  return 0;
}

