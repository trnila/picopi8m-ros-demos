#include <sstream>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "m4core.h"

using namespace boost;


class Sender {
public:
  Sender(asio::serial_port &port): port(port) {}

  void on_message(const std_msgs::String::ConstPtr& msg) {
    asio::write(port, asio::buffer(msg->data));
  }

private:
  asio::serial_port &port;
};

int main(int argc, char **argv) {
  M4Core core;
  core.boot_firmware("/root/catkin_ws/src/m4_hello_tty/m4/build/debug/hello.bin");

  asio::io_service io;
  asio::serial_port port(io);

  // wait until M4 subscribes to the rpmsg channel 'rpmsg-openamp-demo-channel'
  // which will create /dev/ttyRPMSG30
  system::error_code err;
  while(port.open("/dev/ttyRPMSG30", err).value() == system::errc::no_such_file_or_directory) {
    usleep(10 * 1000);
  }
  port.set_option(asio::serial_port_base::baud_rate(115200));

  ros::init(argc, argv, "proxy");
  ros::NodeHandle n;

  Sender sender(port);
  ros::Subscriber sub = n.subscribe("hello_messages", 1000, &Sender::on_message, &sender);

  ros::spin();
  return 0;
}
