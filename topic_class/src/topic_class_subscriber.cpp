#include <ros/ros.h>
#include <topic_class/randomNum.h>

class RandomNum{
public:
  RandomNum()
  : sequence(0), sum(0), last(ros::Time::now())
  {
    sub_ = nh_.subscribe("randomNum", 1000, &RandomNum::callback, this);
  }

  ~RandomNum(){}

  void callback(const topic_class::randomNumConstPtr& ptr){
    ros::Time current = ros::Time::now();
    sequence++;
    sum+=ptr->randint;

    ROS_INFO("seq : %d / sum : %d / delay : %lf", sequence, sum, (double)(current - last).toSec());
    last = current;

    return;
  }
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  int sequence;
  int sum;
  ros::Time last;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "topic_class_subscriber");
  RandomNum randomNum;

  ros::spin();

  return 0;
}