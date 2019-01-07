#include <ros/ros.h>
#include <ros_programming1/rand_msg.h>

class RandNum{
public:
  RandNum() : loop_rate(10), last(ros::Time::now()), seq(0), sum(0)
  {
    sub = nh.subscribe("ros_practice", 1000, &RandNum::cb, this);
  }

  void cb(const ros_programming1::rand_msgConstPtr& ptr){
    ros::Time cur = ros::Time::now();
    double gap = (cur - last).toSec();
    last = cur;

    seq++;

    sum+=ptr->randint;

    ROS_INFO("[%lf][%d] limit : %d / randint : %d/ sum : %d", gap, seq, ptr->limit, ptr->randint, sum);
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Rate loop_rate;
  ros::Time last;
  int seq;
  int sum;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "ros_sub");
  RandNum rn;

  ros::spin();

  return 0;
}
