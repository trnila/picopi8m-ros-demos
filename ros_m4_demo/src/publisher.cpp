#include "ros/ros.h"
#include "ros_m4_demo/Print.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;

  ros::Publisher print_pub = n.advertise<ros_m4_demo::Print>("/ros_m4_demo/print", 1000);

  // throttle loop to 5 Hz
  ros::Rate loop_rate(5);

  ros_m4_demo::Print msg{};
  while (ros::ok()) {
    msg.text = "hello";
    msg.num++; 

    print_pub.publish(msg);

    ros::spinOnce();

    // sleep to keep 5 Hz
    loop_rate.sleep();
  }


  return 0;
}
