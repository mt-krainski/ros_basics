#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_mover");
  ros::NodeHandle nh;
  ros::Publisher turtle_mover_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::Rate loop_rate(1);
  geometry_msgs::Twist twist_msg;
  int phase=0;
  while (ros::ok())
  {
    if(phase == 0)
    {
        twist_msg.linear.x = 1.0;
        twist_msg.angular.z = 0.0;
        phase = 1;
    }
    else if (phase == 1)
    {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 1.0;
        phase = 0;
    }
    turtle_mover_publisher.publish(twist_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
