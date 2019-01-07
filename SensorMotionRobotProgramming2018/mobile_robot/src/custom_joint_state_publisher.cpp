#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

class CustomJointStatePublisher{
public:
    CustomJointStatePublisher() {
        if(!n.getParam("/custom_joint_state_publisher/joint_names", joint_names)) throw std::runtime_error("set joint_names");
        if(!n.getParam("/custom_joint_state_publisher/joint_positions", joint_positions)) throw std::runtime_error("set joint_positions");
        
        joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
        
        n_joint = static_cast<int>(joint_names.size());
        joint_state.name.resize(n_joint);
        joint_state.position.resize(n_joint);
    }

    void pub(){
        for(int joint_idx = 0 ; joint_idx < n_joint; ++joint_idx){
            joint_state.name[joint_idx] = joint_names[joint_idx];
            joint_state.position[joint_idx] = joint_positions[joint_idx];
        }
        joint_state.header.stamp = ros::Time::now();
        joint_pub.publish(joint_state);
    }

private:
    int n_joint;
    ros::NodeHandle n;
    ros::Publisher joint_pub;
    tf::TransformBroadcaster broadcaster;
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    sensor_msgs::JointState joint_state;
    
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_joint_state_publisher");

    CustomJointStatePublisher c;

    ros::Rate loop_rate(30);
    
    while (ros::ok()) {
        c.pub();
        loop_rate.sleep();
    }

    return 0;
}