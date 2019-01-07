#include <ros/ros.h>
#include <led_control/MsgLedControl.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "led_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<led_control::MsgLedControl>("display_led", 1000);
  ros::Rate rate(0.5);

  int led_state = 0;

  while(ros::ok()){
    led_control::MsgLedControl msg;
    msg.data = led_state;
    led_state = (led_state + 1) % 4;

    pub.publish(msg);
    ROS_INFO("state : %d", led_state);
    rate.sleep();
  }

  return 0;
}