#include <ros/ros.h>
#include <std_msgs/Float64.h>

class PID_controller{
public:
  PID_controller()
    : e_pre(0), e_tot(0), cur_position(-1)
  { 
    pid_input_pub = nh_.advertise<std_msgs::Float64>("/pid_input", 1000);
    position_sub = nh_.subscribe("/cur_position", 1000, &PID_controller::positionCb, this);
    nh_.param("/controller/goal", goal, -1.0);
    nh_.param("/controller/kp", kp, -1.0);
    nh_.param("/controller/kd", kd, -1.0);
    nh_.param("/controller/ki", ki, -1.0);
  }

  ~PID_controller(){}
  
  double cal_pid(){
    double cur_sec = ros::Time::now().toSec();
    double dt = cur_sec - last_sec;

    double error = goal - cur_position;
    e_tot += error * dt;

    double e_dot = (error - e_pre) / dt;

    pid_input = kp * error + ki * e_tot + kd * e_dot;

    ROS_INFO("pid_input : %lf",pid_input);

    last_sec = cur_sec;
    e_pre = error;

    return pid_input;
  }

  void pub_pid_input(){
    pid_input_msg.data = cal_pid();
    pid_input_pub.publish(pid_input_msg);

    return;
  }

  void positionCb(const std_msgs::Float64ConstPtr& ptr){
    cur_position = ptr->data;
    ROS_INFO("cur_position : %lf", cur_position);
    ROS_INFO("goal : %lf / kp : %lf / kd : %lf / ki : %lf", goal, kp, kd, ki);

    return;
  }

  bool ready_to_start(){
    return (goal>=0) && (kp>=0) && (kd>=0) && (ki>=0);
  }

  void set_last_sec(double _last){
    last_sec = _last;
  }
private:
  ros::NodeHandle nh_;
  ros::Subscriber position_sub;
  ros::Publisher pid_input_pub;
  
  double goal;
  double kp;
  double kd;
  double ki;

  double last_sec;
  double e_pre;
  double e_tot;

  double cur_position;
  double pid_input;
  std_msgs::Float64 pid_input_msg;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "manipulator_controller");
  PID_controller controller;
  ros::Rate loop_rate(30);

  if( !(controller.ready_to_start()) ){
    ROS_INFO("Cannot get parameters!");
    return -1;
  }

  controller.set_last_sec(ros::Time::now().toSec());
  while(ros::ok()){
    controller.pub_pid_input();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}