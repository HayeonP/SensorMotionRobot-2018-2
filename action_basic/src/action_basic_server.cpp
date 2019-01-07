#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <action_basic/fibonacciAction.h>

class Fibonacci{
public:
  Fibonacci(std::string name)
    : as_(nh_, name, boost::bind(&Fibonacci::cb, this, _1), false), actionName_(name){
      as_.start();
    }

  void cb(const action_basic::fibonacciGoalConstPtr& ptr){
    ros::Rate r(3);
    bool success = true;

    fb_.feedback.clear();
    fb_.feedback.push_back(0);
    fb_.feedback.push_back(1);

    for(int i=1; i<ptr->order; i++){
      if(as_.isPreemptRequested() || !ros::ok()){
        success = false;
        as_.setPreempted();
        break;
      }

      fb_.feedback.push_back(fb_.feedback[i] + fb_.feedback[i-1]);
      as_.publishFeedback(fb_);
      r.sleep();
    }

    if(success){
      rs_.result = fb_.feedback;
      as_.setSucceeded(rs_);
    }

    return;
  }
private:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<action_basic::fibonacciAction> as_;
  action_basic::fibonacciFeedback fb_;
  action_basic::fibonacciResult rs_;
  std::string actionName_;

};

int main(int argc, char** argv){
  ros::init(argc, argv, "action_basic_server");
  Fibonacci fibonacci("fibonacci");

  ros::spin();

  return 0;
}




// #include <ros/ros.h>
// #include <actionlib/server/simple_action_server.h>
// #include <action_basic/fibonacciAction.h>

// class FibonacciAction{
// public:
//   FibonacciAction(std::string name)
//     : as_(nh_, name, boost::bind(&FibonacciAction::callback, this, _1), false), actionName_(name){
      
//       as_.start();
//     }

//   ~FibonacciAction(){}

//   void callback(const action_basic::fibonacciGoalConstPtr& goal){
//     ros::Rate loop_rate(3);
//     bool success = true;

//     feedback_.feedback.clear();
//     feedback_.feedback.push_back(0);
//     feedback_.feedback.push_back(1);

//     for(int i=1; i < goal->order; i++){
//       if(as_.isPreemptRequested() || !ros::ok()){
//         success = false;
//         as_.setPreempted();
//         break;
//       }

//       feedback_.feedback.push_back(feedback_.feedback[i] + feedback_.feedback[i-1]);
//       as_.publishFeedback(feedback_);
//       loop_rate.sleep();
//     }

//     if(success){
//       result_.result = feedback_.feedback;
//       ROS_INFO("Succeeded!");
//       as_.setSucceeded(result_);
//     }

//     return;
//   }
// private:
//   ros::NodeHandle nh_;
//   actionlib::SimpleActionServer<action_basic::fibonacciAction> as_;
//   action_basic::fibonacciFeedback feedback_;
//   action_basic::fibonacciResult result_;
//   std::string actionName_;
// };

// int main(int argc, char** argv){
//   ros::init(argc, argv, "action_basic_server");
//   FibonacciAction fibonacciAction("fibonacci");

//   ros::spin();

//   return 0;
// }