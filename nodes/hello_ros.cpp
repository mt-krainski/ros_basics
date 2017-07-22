#include <ros/ros.h>

int main (int argc, char ** argv){

	ros::init(argc, argv, "hello_ros");

	ros::NodeHandle node_handle;

	ROS_INFO_STREAM("Hello ROS!");
}
