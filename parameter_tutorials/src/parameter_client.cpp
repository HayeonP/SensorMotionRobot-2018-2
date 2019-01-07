#include "ros/ros.h"
#include "parameter_tutorials/SrvTutorial.h"

int main(int argc, char *argv[]){
  ros::init(argc, argv, "parameter_client");

  if(argc!=3){
    ROS_INFO("cmd : rosrun ros_tutorial_service service_client arg0 arg1");
    ROS_INFO("arg0 : double number, arg1 : double number");
    return 1;
  }

  ros::NodeHandle nh;

  ros::ServiceClient parameter_client=
    nh.serviceClient<parameter_tutorials::SrvTutorial>("parameter_tutorial");

  parameter_tutorials::SrvTutorial srv;

  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  if(parameter_client.call(srv)){
    ROS_INFO("send srv, srv.Request.a & b : %ld, %ld", static_cast<long int>(srv.request.a),
        static_cast<long int>(srv.request.b));
    ROS_INFO("receive srv, srv.Response.result : %ld", (long int)srv.response.result);
  }
  else{
    ROS_ERROR("Failed to call service ros_tutorial_srv");
    return 1;
  }

  return 0;
}



