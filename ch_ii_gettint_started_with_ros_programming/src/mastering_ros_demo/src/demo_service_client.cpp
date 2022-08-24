#include "ros/ros.h"
#include "mastering_ros_demo/demo_srv.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_service_client");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::ServiceClient client = 
    n.serviceClient<mastering_ros_demo::demo_srv>("demo_service");
  
  while (ros::ok()) {
    mastering_ros_demo::demo_srv srv;
    srv.request.in = "Sending from here";

    if (client.call(srv)) {
      ROS_INFO("From client [%s], Server says [%s]",
        srv.request.in.c_str(), srv.response.out.c_str());
    }
    else {
      ROS_ERROR("Fail to call service");
      return 1;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}