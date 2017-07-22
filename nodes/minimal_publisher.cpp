#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello ROS " << count;
    msg.data = ss.str();
    ROS_INFO_STREAM(msg.data);
    chatter_pub.publish(msg);
    ros::spinOnce();
    count++;
    loop_rate.sleep();
  }
  return 0;
}
