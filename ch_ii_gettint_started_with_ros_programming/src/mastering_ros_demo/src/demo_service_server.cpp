#include "ros/ros.h"
#include "mastering_ros_demo/demo_srv.h"


bool demo_service_callback(mastering_ros_demo::demo_srv::Request &req,
  mastering_ros_demo::demo_srv::Response &res
) {
  res.out = "Received here: " + req.in;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_service_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = 
    n.advertiseService("demo_service", demo_service_callback);

  ROS_INFO("Ready to receive from client.");
  ros::spin();
}