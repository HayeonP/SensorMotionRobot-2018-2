#include <ros/ros.h>
#include <service_basic/serviceBasic.h>


bool callback(service_basic::serviceBasic::Request& req, service_basic::serviceBasic::Response& res){
  res.result = req.a + req.b;

  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "service_basic_server");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("serviceBasic", callback);

  ROS_INFO("Service server start!");
  ros::spin();

  return 0;
}