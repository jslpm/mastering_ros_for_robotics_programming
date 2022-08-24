#include <iostream>
#include "ros/ros.h"
#include "mastering_ros_demo/demo_msg.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "demo_msg_publisher");
  ros::NodeHandle node_obj;
  ros::Publisher number_publisher = 
    node_obj.advertise<mastering_ros_demo::demo_msg>("demo_msg", 10);
  ros::Rate loop_rate(10);
  int number_count = 0;

  while(ros::ok()) {
    mastering_ros_demo::demo_msg msg;
    msg.greeting = "Hello ros world!";
    msg.number = number_count;
    ROS_INFO("%s %d", msg.greeting.c_str(), msg.number);
    //ROS_INFO_STREAM(msg.greeting << " " << msg.number);
    number_publisher.publish(msg);
    loop_rate.sleep();
    ++number_count;
  }
}