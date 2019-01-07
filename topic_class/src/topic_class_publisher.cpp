#include <ros/ros.h>
#include <topic_class/randomNum.h>
#include <cstdlib>

int main(int argc, char** argv){
  ros::init(argc, argv, "topic_class_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<topic_class::randomNum>("randomNum", 1000);
  ros::Rate loop_rate(5);

  topic_class::randomNum msg;
  int limit;

  while(ros::ok()){
    nh.param("/limit", limit, 10);
    msg.limit = limit;

    msg.randint = rand()%limit;

    ROS_INFO("limit : %d / randint : %d", msg.limit, msg.randint);

    pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}