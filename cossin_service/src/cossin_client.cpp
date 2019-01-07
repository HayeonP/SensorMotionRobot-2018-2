#include <ros/ros.h>
#include <cossin_service/SrvAngle.h>
#include <stdlib.h>

int main(int argc, char** argv){
  if(argc < 2){
    ROS_INFO("You need angle at argv[1]");
    return -1;
  }

  ros::init(argc, argv, "cossin_client");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<cossin_service::SrvAngle>("SrvAngle");
  
  cossin_service::SrvAngle srv;

  srv.request.angle = (float)atof(argv[1]);

  if(client.call(srv)){
    ROS_INFO("angle : %f / cos : %f / sin : %f", (float)(srv.request.angle), 
    (float)(srv.response.cosValue), (float)(srv.response.sinValue));
  }
  else{
    ROS_ERROR("Cannot contact to the server!");
    return -1;
  }
  
  return 0;
}