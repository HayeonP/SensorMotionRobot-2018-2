#include <ros/ros.h>
#include <service_basic/serviceBasic.h>

int main(int argc, char** argv){
  
  if(argc != 3){
    ROS_INFO("ERROR : service_basic {number1} {number2} ");
    return -1;
  }

  ros::init(argc, argv, "service_basic_client");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<service_basic::serviceBasic>("serviceBasic");
  
  service_basic::serviceBasic srv;

  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);

  if(client.call(srv)){
    ROS_INFO("a : %ld, b : %ld, result %ld", (long int)(srv.request.a), 
      (long int)(srv.request.b), (long int)(srv.response.result));
  }
  else{
    ROS_ERROR("Cannot contact to the server!");
    return -1;
  }
  
  return 0;
}