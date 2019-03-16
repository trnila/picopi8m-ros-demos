#include "ros/ros.h"
#include "../benchmark.h"
#include "std_msgs/UInt32.h"

uint32_t count = 10000;
uint64_t *benchmark;
int ping = 0;
ros::Publisher pub;

void hello_cb(const std_msgs::UInt32 &msg) {
  benchmark[ping] = benchmark_stop();
  std_msgs::UInt32 val;
  val.data = ping++;

  if(ping < count) {
    benchmark_start();
    pub.publish(val);
  } else {
    for(int i = 0; i < count; i++) {
      printf("%d\n", benchmark[i]);
    }
    exit(1);
  }
}

int main(int argc, char **argv) {
  if(argc >= 2) {
    count = atoi(argv[1]);
  }

  benchmark = new uint64_t[count];

  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/benchmark/m4", 1000, hello_cb);


  pub = n.advertise<std_msgs::UInt32>("/benchmark/linux", 1000);
  std_msgs::UInt32 val;
  val.data = 0;

  sleep(1);
  pub.publish(val);
  ros::spin();
  return 0;
}
