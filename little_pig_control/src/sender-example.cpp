#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sender");

    ros::NodeHandle n;

    ros::Publisher sender_pub = n.advertise<std_msgs::String>("sender", 1000);

    ros::Rate loop_rate(10);

    int count = 0;

    while(ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;

        ss << "hello little pig" << count;

        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        sender_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

        count++;
    }

    return 0;
}