#include "ros/ros.h"
#include "std_msgs/String.h"

//callback function define
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    //print msg
    ROS_INFO("I hear:[%s]",msg->data.c_str());
}

int  main(int argc, char **argv)
{
    //ROS initialize
    ros::init(argc, argv, "string_subscriber" );

    //NodeHandle create
    ros::NodeHandle n;

    //create Subscriber and subscribe to the chatter topic and function chatterCallback
    ros::Subscriber sub=n.subscribe("chatter",1000,chatterCallback);

    //wait
    ros::spin();

    return 0;
}