#include <ros/ros.h>
#include <ros_programming1/rand_msg.h>
#include <cstdlib>

int main(int argc, char **argv){
  ros::init(argc, argv, "ros_pub");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<ros_programming1::rand_msg>("ros_practice", 1000);
  ros::Rate loop_rate(10);
  
  int limit, randint;
  while(ros::ok()){
    ros_programming1::rand_msg msg;

    nh.param("/limit", limit, 10);
    randint = rand()%limit;

    msg.limit = limit;
    msg.randint = randint;
    
    ROS_INFO("limit : %d / randint : %d", limit, randint);
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
