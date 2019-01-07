#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>

class EndeffectorExtractor{

public:
  EndeffectorExtractor(){
    endeffector_pub = nh.advertise<geometry_msgs::PoseStamped>("endeffector", 10);
    joint_sub = nh.subscribe("darray", 10, &EndeffectorExtractor::jointCB, this);

    homogeneous_matrix.resize(4);
    for(auto& vec : homogeneous_matrix)
      vec.resize(4);
  }

  void jointCB(const std_msgs::Float64MultiArrayPtr& ptr){
    darray.data = ptr->data;
    ROS_INFO("----------");
    for(auto& d : darray.data)
      ROS_INFO("%lf", d);
    calcPos();
    ROS_INFO("pose of endeffetor : %lf %lf %lf", end_pos.position.x, end_pos.position.y, end_pos.position.z);
  }

  void calcHomogeneousMatrix(const std_msgs::Float64MultiArray& darray){
    homogeneous_matrix[0][0] = 1;
    homogeneous_matrix[0][1] = 0;
    homogeneous_matrix[0][2] = 0;
    homogeneous_matrix[0][3] = 0;

    homogeneous_matrix[1][0] = 0;
    homogeneous_matrix[1][1] = 1;
    homogeneous_matrix[1][2] = 0;
    homogeneous_matrix[1][3] = 0;

    homogeneous_matrix[2][0] = 0;
    homogeneous_matrix[2][1] = 0;
    homogeneous_matrix[2][2] = 1;
    homogeneous_matrix[2][3] = 0;

    homogeneous_matrix[3][0] = 0;
    homogeneous_matrix[3][1] = 0;
    homogeneous_matrix[3][2] = 0;
    homogeneous_matrix[3][3] = 1;
  }

  void calcPos(){
    calcHomogeneousMatrix(darray);
    end_pos.position.x = 
      homogeneous_matrix[0][0]*cur_pos.position.x
        + homogeneous_matrix[0][1]*cur_pos.position.y
          + homogeneous_matrix[0][2]*cur_pos.position.z
            + homogeneous_matrix[0][3]*1;

      homogeneous_matrix[1][0]*cur_pos.position.x
        + homogeneous_matrix[1][1]*cur_pos.position.y
          + homogeneous_matrix[1][2]*cur_pos.position.z
            + homogeneous_matrix[1][3]*1;

      homogeneous_matrix[2][0]*cur_pos.position.x
        + homogeneous_matrix[2][1]*cur_pos.position.y
          + homogeneous_matrix[2][2]*cur_pos.position.z
            + homogeneous_matrix[2][3]*1;
  }

private:
  ros::NodeHandle nh;
  ros::Subscriber joint_sub;
  ros::Publisher endeffector_pub;
  std_msgs::Float64MultiArray darray;

  std::vector< std::vector<double> > homogeneous_matrix;
  geometry_msgs::Pose end_pos, cur_pos;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "endeffector_extractor");

  EndeffectorExtractor e;

  ros::spin();
  return 0;
}