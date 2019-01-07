#include <ros/ros.h>
#include <topic_basic/topicBasicMsg.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "topic_basic_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<topic_basic::topicBasicMsg>("topicBasicMsg", 1000);
  ros::Rate loop_rate(5);

  topic_basic::topicBasicMsg msg;
  int count = 1;
  std::string id;
  
  while(ros::ok()){
    std::stringstream ss;

    nh.param("/chatter_id", id, std::string("no id string"));
    msg.id = id;

    ss << "Hello Wrold" << count;
    msg.data = ss.str();

    ROS_INFO("[ %s ] : %s", msg.id.c_str(), msg.data.c_str());
    pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();

    count++;
  }

  return 0;
}