#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


class ProjectOdom{
public:
    ProjectOdom():seq(0), nh(){
        OdomSub = nh.subscribe("/odom", 10, &ProjectOdom::OdomCB, this);
    }

    void OdomCB(const nav_msgs::OdometryConstPtr ptr){
        ros::Time cur = ros::Time::now();
        
        double x = ptr->pose.pose.position.x;
        double y = ptr->pose.pose.position.y;
        tf::Quaternion q; tf::quaternionMsgToTF(ptr->pose.pose.orientation, q);
        double x_dot = ptr->twist.twist.linear.x;
        double y_dot = ptr->twist.twist.linear.y;
        double theta_dot = ptr->twist.twist.angular.z;
        
        //generate tf msg
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, cur, "custom_odom", "base_footprint"));
    }

private://odometry members
    uint seq;
private://ros dependency
    ros::Subscriber OdomSub;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "tf_project");

    ProjectOdom d;
    ros::spin();
    return 0;
}