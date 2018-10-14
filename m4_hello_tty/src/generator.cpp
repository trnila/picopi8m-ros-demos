#include <sstream>
#include <boost/asio/serial_port.hpp> 
#include <boost/asio.hpp> 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "m4core.h"

using namespace boost;


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

  int i = 0;
  for(;;) {
    char buffer[128];
    int size = snprintf(buffer, sizeof(buffer), "hello world %d", i);
    asio::write(port, asio::buffer(buffer, size));
    sleep(1);
    i++;
  }

  return 1;
}
