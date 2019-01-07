#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


class OdomExtractor{
public:
    OdomExtractor():seq(0), nh(){
        linkStateSub = nh.subscribe("/gazebo/link_states", 10, &OdomExtractor::LinkStatesCB, this);
        odomPub = nh.advertise<nav_msgs::Odometry>("/custom_odom", 10);
    }

    void LinkStatesCB(const gazebo_msgs::LinkStatesConstPtr ptr){
        int idx;
        auto it = std::find(ptr->name.begin(), ptr->name.end(), "MobileRobot::base_footprint");
        if (it != ptr->name.end()) idx = it - ptr->name.begin();
        else throw std::runtime_error("can not find MobileRobot::base_footprint");
        
        ros::Time cur = ros::Time::now();
        
        double x = ptr->pose[idx].position.x;
        double y = ptr->pose[idx].position.y;
        tf::Quaternion q; tf::quaternionMsgToTF(ptr->pose[idx].orientation, q);
        double x_dot = ptr->twist[idx].linear.x;
        double y_dot = ptr->twist[idx].linear.y;
        double theta_dot = ptr->twist[idx].angular.z;
        
        //generate tf msg
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, cur, "custom_odom", "custom_base"));

        //publish odom
        nav_msgs::Odometry odom;
        odom.header.seq = seq++;
        odom.header.stamp = cur;
        odom.header.frame_id = "custom_odom";
        odom.child_frame_id = "base_odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = ptr->pose[idx].orientation;
        odom.twist.twist.linear.x = x_dot;
        odom.twist.twist.linear.y = y_dot;
        odom.twist.twist.angular.z = theta_dot;

        odomPub.publish(odom);
    }

private://odometry members
    uint seq;
private://ros dependency
    ros::Publisher odomPub;
    ros::Subscriber linkStateSub;
    ros::Subscriber diffVelSub;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "odom_extactor");

    OdomExtractor d;
    ros::spin();
    return 0;
}