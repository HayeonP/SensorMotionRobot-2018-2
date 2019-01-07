#include <ros/ros.h>
#include <service_tutorial/test_srv.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "client");

  if(argc<3){
    ROS_INFO("Error");
    return 1;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<service_tutorial::test_srv>("ros_srv");

  service_tutorial::test_srv srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  if(client.call(srv)){
    ROS_INFO("a : %ld / b : %ld / result : %ld", (long int)srv.request.a, (long int)srv.request.b, (long int)srv.response.result);
  }
  else{
    ROS_ERROR("ERROR");
  }

  return 0;
}
