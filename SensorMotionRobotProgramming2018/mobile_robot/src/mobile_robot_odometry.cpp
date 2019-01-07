#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <algorithm>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <diff_velocity_msgs/diffVelocity.h>
#include <thread>

class DiffDriveOdometry{
public:
    DiffDriveOdometry():seq(0), nh(), x(0.0), y(0.0), theta(0.0), left_vel_vector(), right_vel_vector(){
        last = ros::Time::now();

        if (!nh.getParam("/mobile_robot_odometry/leftwheel_linkname", wheel1_linkname)) 
            throw std::runtime_error("set leftwheel_linkname");
        if (!nh.getParam("/mobile_robot_odometry/rightwheel_linkname", wheel3_linkname)) 
            throw std::runtime_error("set rightwheel_linkname");
        if (!nh.getParam("/mobile_robot_odometry/base_link_id", base_link_id)) 
            throw std::runtime_error("set base_link_id");
        if (!nh.getParam("/mobile_robot_odometry/odom_link_id", odom_link_id)) 
            throw std::runtime_error("set odom_link_id");
        if (!nh.getParam("/mobile_robot_odometry/separation_length", separation_length)) 
            throw std::runtime_error("set separation_length");
            
        linkStateSub = nh.subscribe("/gazebo/link_states", 10, &DiffDriveOdometry::calcWheelVelocityGazeboCB, this);
        odomPub = nh.advertise<nav_msgs::Odometry>("/custom_odom", 10);
        diffVelSub = nh.subscribe("/diff_velocity", 10, &DiffDriveOdometry::calcWheelVelocityCB, this);
        
        std::thread([&](){
            ros::Rate r(50);
            while(ros::ok()){
                broadcastTransform();
                r.sleep();
            }
        }).detach();
    }

    void calcWheelVelocityGazeboCB(const gazebo_msgs::LinkStatesConstPtr& ptr){
        //find index
        int wheel1_idx, wheel3_idx;
        auto it1 = std::find(ptr->name.begin(), ptr->name.end(), wheel1_linkname);
        if (it1 != ptr->name.end()) wheel1_idx = it1 - ptr->name.begin();
        else throw std::runtime_error("can not find wheel1_linkname");
        auto it3 = std::find(ptr->name.begin(), ptr->name.end(), wheel3_linkname);
        if (it3 != ptr->name.end()) wheel3_idx = it3 - ptr->name.begin();
        else throw std::runtime_error("can not find wheel3_linkname");
        
        //calculate velocity
        left_wheel_velocity = sqrt(
            std::pow(ptr->twist[wheel1_idx].linear.x, 2) + 
            std::pow(ptr->twist[wheel1_idx].linear.y, 2) // ignore vel_z
        );
        //detect reversed velocity with inner product.
        //if the direction of the rebot change, the result of inner product of velocity vector of each wheel
        //have negative value.
        right_wheel_velocity = sqrt(
            std::pow(ptr->twist[wheel3_idx].linear.x, 2) + 
            std::pow(ptr->twist[wheel3_idx].linear.y, 2) // ignore vel_z
        );
        double left_inner_product = ptr->twist[wheel1_idx].linear.x*left_vel_vector.x
            + ptr->twist[wheel1_idx].linear.y*left_vel_vector.y;
        if (left_inner_product < 0) left_wheel_velocity *= -1;
        double right_inner_product = ptr->twist[wheel3_idx].linear.x*right_vel_vector.x
            + ptr->twist[wheel3_idx].linear.y*right_vel_vector.y;
        if (right_inner_product < 0) right_wheel_velocity *= -1;

        left_vel_vector.x = ptr->twist[wheel1_idx].linear.x;
        left_vel_vector.y = ptr->twist[wheel1_idx].linear.y;
        right_vel_vector.x = ptr->twist[wheel3_idx].linear.x;
        right_vel_vector.y = ptr->twist[wheel3_idx].linear.y;
    }

    void broadcastTransform(){
        ros::Time cur = ros::Time::now();

        //calc odometry
        double v = (left_wheel_velocity + right_wheel_velocity) / 2;
        double x_dot = v * std::cos(theta);
        double y_dot = v * std::sin(theta);
        double theta_dot = (right_wheel_velocity - left_wheel_velocity) / separation_length;

        double dt = (cur - last).toSec();
        double dx = x_dot * dt;
        double dy = y_dot * dt;
        double dtheta = theta_dot * dt;
        x += dx;
        y += dy;
        theta += dtheta;

        //check Nan
        if (x != x) x = 0;
        if (y != y) y = 0;
        if (theta != theta) theta = 0;

        //generate tf msg
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, cur, odom_link_id, base_link_id));

        //publish odom
        nav_msgs::Odometry odom;
        odom.header.seq = seq++;
        odom.header.stamp = cur;
        odom.header.frame_id = odom_link_id;
        odom.child_frame_id = base_link_id;

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        odom.twist.twist.linear.x = x_dot;
        odom.twist.twist.linear.y = y_dot;
        odom.twist.twist.angular.z = theta_dot;

        odomPub.publish(odom);
        last = cur;
    }
    
    void calcWheelVelocityCB(const diff_velocity_msgs::diffVelocityConstPtr& ptr){
        left_wheel_velocity = ptr->left_wheel_velocity;
        right_wheel_velocity = ptr->right_wheel_velocity;
    }

private://paramter members
    std::string wheel1_linkname;
    std::string wheel3_linkname;
    std::string base_link_id;
    std::string odom_link_id;
    double separation_length;
private://odometry members
    double x, y, theta;
    ros::Time last;
    uint seq;
private://shared variable
    double left_wheel_velocity;
    double right_wheel_velocity;
private://ros dependency
    ros::Publisher odomPub;
    ros::Subscriber linkStateSub;
    ros::Subscriber diffVelSub;
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
private:
    struct Vec2d{double x; double y;};
    Vec2d left_vel_vector, right_vel_vector;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "mobile_robot_odometry");

    DiffDriveOdometry d;
    ros::spin();
    return 0;
}