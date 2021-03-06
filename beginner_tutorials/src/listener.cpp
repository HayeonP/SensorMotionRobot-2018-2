#include "ros/ros.h"
#include "beginner_tutorials/simple_msg.h"

void chatterCallback(const beginner_tutorials::simple_msgConstPtr& ptr)
{
    ROS_INFO("I heard from %s: [%s]", ptr->id.c_str(), ptr->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::spin();

    return 0;
}
