#include <ros/ros.h>
#include <diff_velocity_msgs/diffVelocity.h>
#include <geometry_msgs/Twist.h>

class CmdVelToDiffDrive{
public:
    CmdVelToDiffDrive() {
        if(!nh.getParam("/cmd_vel_to_diff_drive/separation_length", separation_length))
            throw std::runtime_error("set separation_length!");
        pub = nh.advertise<diff_velocity_msgs::diffVelocity>("diff_velocity", 10);
        sub = nh.subscribe("/cmd_vel", 10, &CmdVelToDiffDrive::cmd_vel_CB, this);
    }
    void cmd_vel_CB(const geometry_msgs::TwistConstPtr& ptr){
        double v = ptr->linear.x;
        double w = ptr->angular.z;

        double right_wheel_velocity = (2*v + w*separation_length) / 2;
        double left_wheel_velocity = (2*v - w*separation_length) / 2;

        diff_velocity_msgs::diffVelocity msg;
        msg.left_wheel_velocity = left_wheel_velocity;
        msg.right_wheel_velocity = right_wheel_velocity;

        pub.publish(msg);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    double separation_length;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "cmd_vel_to_diff_drive");
    CmdVelToDiffDrive c;

    ros::spin();
    return 0;
}