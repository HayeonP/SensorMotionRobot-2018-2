#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <goal_point_collector/name.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <errno.h>

class GoalPointCollector{
public:
  GoalPointCollector(){
    sub_ = nh_.subscribe("/move_base_simple/goal", 1, &GoalPointCollector::goalCb, this);
    goal_list.clear();
  }

  void goalCb(const geometry_msgs::PoseStampedPtr& ptr){
    goal_list.push_back(*ptr);
    ROS_INFO("RECEIVE!");
    return;
  }

  bool saveCb(goal_point_collector::name::Request& req, goal_point_collector::name::Response& res){
    std::string filename = std::string(req.filename);

    std::ofstream fout;
    fout.open(filename);
    
    if(!fout.is_open()){
      ROS_ERROR("%s", strerror(errno));
      return false;
    }

    if(goal_list.empty()){
      ROS_WARN("Goal list is empty");
      return false;
    }

    fout << "n_data : "<< goal_list.size() <<"\n";
    
    fout << "goal_pos_x : ["; 
    for (auto it = goal_list.begin(); it != goal_list.end(); ++it){
      fout<< (*it).pose.position.x;
      
      if((it+1) == goal_list.end()){
        fout << "]\n";
        break;
      }

      fout<< ",";
    }

    fout << "goal_pos_y : ["; 
    for (auto it = goal_list.begin(); it != goal_list.end(); ++it){
      fout<< (*it).pose.position.y;
      
      if((it+1) == goal_list.end()){
        fout << "]\n";
        break;
      }

      fout<< ",";
    }

    fout << "goal_pos_z : ["; 
    for (auto it = goal_list.begin(); it != goal_list.end(); ++it){
      fout<< (*it).pose.position.z;
      
      if((it+1) == goal_list.end()){
        fout << "]\n";
        break;
      }

      fout<< ",";
    }
    
    fout << "goal_ori_x : ["; 
    for (auto it = goal_list.begin(); it != goal_list.end(); ++it){
      fout<< (*it).pose.orientation.x;
      
      if((it+1) == goal_list.end()){
        fout << "]\n";
        break;
      }

      fout<< ",";
    }

    fout << "goal_ori_y : ["; 
    for (auto it = goal_list.begin(); it != goal_list.end(); ++it){
      fout<< (*it).pose.orientation.y;
      
      if((it+1) == goal_list.end()){
        fout << "]\n";
        break;
      }

      fout<< ",";
    }

    fout << "goal_ori_z : ["; 
    for (auto it = goal_list.begin(); it != goal_list.end(); ++it){
      fout<< (*it).pose.orientation.z;
      
      if((it+1) == goal_list.end()){
        fout << "]\n";
        break;
      }

      fout<< ",";
    }

    fout << "goal_ori_w : ["; 
    for (auto it = goal_list.begin(); it != goal_list.end(); ++it){
      fout<< (*it).pose.orientation.w;
      
      if((it+1) == goal_list.end()){
        fout << "]\n";
        break;
      }

      fout<< ",";
    }

    ROS_INFO("Saved in path %s", filename.c_str());

    goal_list.clear();

    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::vector<geometry_msgs::PoseStamped> goal_list;
  ros::ServiceServer server_ = nh_.advertiseService("goal_save", &GoalPointCollector::saveCb, this);

};

int main(int argc, char** argv){
  ros::init(argc, argv, "goal_point_collector");
  GoalPointCollector goalPointCollector;

  ros::spin();

  return 0;
}