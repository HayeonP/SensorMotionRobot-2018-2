#include "ros/ros.h"
#include "std_msgs/Byte.h"


static int frequency = -1;
static int startByte = -1;

int main(int argc, char **argv){
	ros::init(argc, argv, "led_pub");	
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<std_msgs::Byte>("led_out", 1000);

	nh.param("/led_pub/frequency", frequency, -1);
  nh.param("/led_pub/startByte", startByte, -1);
  if(frequency == -1 || startByte == -1) {
    ROS_INFO("No parameter!");
    return 0;
  }

  ros::Rate loop_rate = frequency;
  int led = startByte;
  int pre_startByte = startByte;

	while(ros::ok()){
    loop_rate.sleep();
		
		std_msgs::Byte msg;
    /*
    nh.param("/led_pub/frequency", frequency, -1);
  	nh.param("/led_pub/startByte", startByte, -1);
    
    ROS_INFO("preStartByte : %d", pre_startByte);
    if(startByte != pre_startByte){
      ROS_INFO("CHANGE");
      pre_startByte = startByte;
      led = startByte;
    }
    */
		loop_rate = frequency;
    msg.data = pow(2, led%4);

    pub.publish(msg);		
    
		ROS_INFO("%d / %d", frequency, msg.data);
    
    led++;

	}
}
