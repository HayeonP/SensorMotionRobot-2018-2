#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>

class DiffDriveOdometry{
public:
    DiffDriveOdometry(){
        gzSub = nh.subscribe("/gazebo/link_states", 
            10, &DiffDriveOdometry::gazeboCB, this);
    }
    void gazeboCB(const gazebo_msgs::LinkStatesConstPtr & ptr){
        //std::vector<std::string> name = ptr->name;
        auto& name = ptr->name;
        auto iter1 = std::find(name.begin(), name.end(), wheel1_name);
        if (iter1 == name.end()) {
            ROS_ERROR("wheel1 link not found!");
            exit(1);
        }
        auto iter3 = std::find(name.begin(), name.end(), wheel3_name);
        if (iter3 == name.end()) {
            ROS_ERROR("wheel1 link not found!");
            exit(1);
        }        

        int wheel1_idx = iter1 - name.begin();
        int wheel3_idx = iter3 - name.begin(); 
        
        left_wheel_velocity = sqrt(
            pow(ptr->twist[wheel1_idx].linear.x, 2) +
            pow(ptr->twist[wheel1_idx].linear.y, 2)
        );
        right_wheel_velocity = sqrt(
            pow(ptr->twist[wheel3_idx].linear.x, 2) +
            pow(ptr->twist[wheel3_idx].linear.y, 2)
        );

        ROS_INFO("vel r, l : %lf %lf", left_wheel_velocity, right_wheel_velocity);
    }
private:
    const std::string wheel1_name = "MobileRobot::wheel_1";
    const std::string wheel3_name = "MobileRobot::wheel_3";
    ros::NodeHandle nh;
    ros::Subscriber gzSub;
private: // gazebo CB members
    double left_wheel_velocity;
    double right_wheel_velocity;
    
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "gazebo_sub_test");

    DiffDriveOdometry d;

    ros::spin();
    return 0;
}