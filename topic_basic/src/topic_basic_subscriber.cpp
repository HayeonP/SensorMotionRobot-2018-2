#include <ros/ros.h>
#include <topic_basic/topicBasicMsg.h>

void callback(const topic_basic::topicBasicMsgConstPtr& ptr){
  ROS_INFO("[ %s ] : %s", ptr->id.c_str(), ptr->data.c_str());

  return;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "topic_basic_subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("topicBasicMsg", 1000, callback );

  ros::spin();

  return 0;
}