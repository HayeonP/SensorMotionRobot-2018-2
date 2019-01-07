#include <ros/ros.h>
#include <cossin_service/SrvAngle.h>
#include <math.h>


bool callback(cossin_service::SrvAngle::Request& req, cossin_service::SrvAngle::Response& res){
  res.cosValue = (float)( cos( (double)(req.angle) ) );
  res.sinValue = (float)( sin( (double)(req.angle) ) );

  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cossin_server");
  ros::NodeHandle nh;
  ros::ServiceServer server = nh.advertiseService("SrvAngle", callback);

  ROS_INFO("Service server start!");
  ros::spin();

  return 0;
}