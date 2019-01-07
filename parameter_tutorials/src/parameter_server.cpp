#include "ros/ros.h"
#include "parameter_tutorials/SrvTutorial.h"

enum {PLUS=1, MINUS, MULTIPLICATION, DIVISION};

int g_operator = PLUS;

bool calculation(parameter_tutorials::SrvTutorial::Request &req,
    parameter_tutorials::SrvTutorial::Response &res){
  ROS_INFO("g_operator : %d", g_operator);

  switch(g_operator){
    case PLUS:
      res.result = req.a + req.b; break;
    case MINUS:
      res.result = req.a - req.b; break;
    case MULTIPLICATION:
      res.result = req.a * req.b; break;
    case DIVISION:
      res.result = (req.b==0) ? 0 : req.a / req.b; break;
    default:
      ROS_WARN("UNKNOWN OPERATOR TYPE. It must be 1, 2, 3, 4. The value of operator is %d", g_operator);
      break;
  }


  ROS_INFO("request : x = %ld, y = %ld, operator : %d", static_cast<long int>(req.a), static_cast<long int>(req.b), g_operator);
  ROS_INFO("sending back response : %ld", static_cast<long int>(res.result));

  return true;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "parameter_server");
  ros::NodeHandle nh;

  nh.setParam("calculation_method", g_operator);
  ros::ServiceServer parameter_server = nh.advertiseService("parameter_tutorial", calculation);
 
  ROS_INFO("ready srv server!");
  ros::Rate loop_rate(10);

  while(ros::ok()){
    nh.getParam("calculation_method", g_operator);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;

}
