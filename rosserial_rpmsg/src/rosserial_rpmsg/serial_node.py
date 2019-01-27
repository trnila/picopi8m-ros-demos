#!/usr/bin/env python

import rospy
from rosserial_python import SerialClient
from serial import SerialException, SerialTimeoutException
from time import sleep
import os
import sys


class M4charStream:
    def __init__(self):
        self.fd = os.open("/dev/m4char", os.O_RDWR)
        self.buffer = None

    def read(self, length):
        out = ""
        r = 0

        while len(out) < length:
            if self.buffer is None or len(self.buffer) <= 0:
                self.buffer = os.read(self.fd, 512)

            out += self.buffer[:length - r]
            self.buffer = self.buffer[length - r:]
            r = len(out)

        return out

    def write(self, data):
        try:
            return os.write(self.fd, data)
        except Exception as e:
            raise SerialTimeoutException(e)

    def inWaiting(self):
        return 1

    def flushInput(self):
        pass

    def flushOutput(self):
        pass

if __name__=="__main__":
    rospy.init_node("serial_node")

    port = rospy.get_param('~port','/dev/ttyRPMSG30')
    sys.argv = rospy.myargv(argv=sys.argv)
    if len(sys.argv) >= 2 :
        port  = sys.argv[1]

    if port == "/dev/m4char":
        port = M4charStream()

    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Opening {}".format(port))

            client = SerialClient(port)
            client.run()
        except KeyboardInterrupt:
            break
        except SerialException:
            sleep(1.0)
            continue
        except OSError:
            sleep(1.0)
            continue
