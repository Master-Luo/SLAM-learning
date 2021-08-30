/*  发布话题，消息类型为string */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"


int main(int argc, char **argv)
{
    //ROS Initialize
    ros::init(argc, argv, "string_publisher");

    //create NodeHandle
    ros::NodeHandle n;

    //create Publisher and topic, class is std_msg::String
    ros::Publisher  chatter_pub = n.advertise<std_msgs::String>("chatter",1000);

    //set loop
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        //std_msgs::String Initialize
        std_msgs::String msg;
        std::stringstream ss;
        ss<<count;
        msg.data=ss.str();

        //topic publich
        ROS_INFO(msg.data.c_str());
        chatter_pub.publish(msg);

        //wait
        loop_rate.sleep();
        ++count;

    }
    return 0;
}