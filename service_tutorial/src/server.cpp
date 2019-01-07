#include <ros/ros.h>
#include <service_tutorial/test_srv.h>

bool cb(service_tutorial::test_srv::Request& req, service_tutorial::test_srv::Response& res){
  res.result = req.a + req.b;
  ROS_INFO("a : %ld / b : %ld / result : %ld", (long int)req.a, (long int)req.b, (long int)res.result);

  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "server");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("ros_srv", cb);

  ros::spin();

  return 0;
}
