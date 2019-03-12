#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "ros_m4_demo/SetSigma.h"
#include "ros_m4_demo/Pause.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "pause");
  ros::NodeHandle n("pause");

  ros::ServiceClient pause_svc = n.serviceClient<ros_m4_demo::Pause>("/ros_m4_demo/pause");
  ros_m4_demo::Pause paused, resume;
  paused.request.pause = 1;
  resume.request.pause = 0;

  if(!pause_svc.call(paused)) {
    printf("failed to issue pause call\n");
  }
  printf("pause count: %d\n", paused.response.count);

  if(!pause_svc.call(resume)) {
    printf("failed to issue pause call\n");
  }
  printf("resume count: %d\n", resume.response.count);


  ros::ServiceClient sigma_svc = n.serviceClient<ros_m4_demo::SetSigma>("/ros_m4_demo/set_sigma"); 
  ros_m4_demo::SetSigma::Request req;
  ros_m4_demo::SetSigma::Response resp;
  if(!sigma_svc.call(req, resp)) {
    printf("failed to issue SetSigma call\n");
  }

  return 0;
}
