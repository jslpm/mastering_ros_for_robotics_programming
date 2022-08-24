#include <iostream>
#include "ros/ros.h"
#include "mastering_ros_demo/demo_msg.h"

void msg_callback(const mastering_ros_demo::demo_msg::ConstPtr& msg) {
  ROS_INFO_STREAM("Received: [" << msg->greeting << " "
    << msg->number << "]");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_msg_subscriber");
  ros::NodeHandle node_obj;
  ros::Subscriber number_subscriber = 
    node_obj.subscribe("demo_msg", 10, msg_callback);
  ros::spin();
}