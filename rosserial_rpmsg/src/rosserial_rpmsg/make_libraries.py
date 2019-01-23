#!/usr/bin/env python
THIS_PACKAGE = "rosserial_rpmsg"

import os
import distutils.dir_util
import rospkg
import rosserial_client
from rosserial_client.make_library import *

ROS_TO_EMBEDDED_TYPES = {
    'bool'    :   ('bool',              1, PrimitiveDataType, []),
    'byte'    :   ('int8_t',            1, PrimitiveDataType, []),
    'int8'    :   ('int8_t',            1, PrimitiveDataType, []),
    'char'    :   ('uint8_t',           1, PrimitiveDataType, []),
    'uint8'   :   ('uint8_t',           1, PrimitiveDataType, []),
    'int16'   :   ('int16_t',           2, PrimitiveDataType, []),
    'uint16'  :   ('uint16_t',          2, PrimitiveDataType, []),
    'int32'   :   ('int32_t',           4, PrimitiveDataType, []),
    'uint32'  :   ('uint32_t',          4, PrimitiveDataType, []),
    'int64'   :   ('int64_t',           8, PrimitiveDataType, []),
    'uint64'  :   ('uint64_t',          8, PrimitiveDataType, []),
    'float32' :   ('float',             4, PrimitiveDataType, []),
    'float64' :   ('float',             4, AVR_Float64DataType, []),
    'time'    :   ('ros::Time',         8, TimeDataType, ['ros/time']),
    'duration':   ('ros::Duration',     8, TimeDataType, ['ros/duration']),
    'string'  :   ('char*',             0, StringDataType, []),
    'Header'  :   ('std_msgs::Header',  0, MessageDataType, ['std_msgs/Header'])
}

if len(sys.argv) < 2:
    print("Usage: make_libraries.py output_directory")
    exit(1)
   
rospack = rospkg.RosPack()
dest = os.path.join(sys.argv[1], "ros_lib")
src = os.path.join(rospack.get_path(THIS_PACKAGE), "src", "ros_lib")

# copy rpmsg hardware-specific files
distutils.dir_util.copy_tree(src, dest)

# copy generic rosserial files - may fail if already exists
try:
    rosserial_client_copy_files(rospack, dest)
except OSError as e:
    print(e)

# generate messages and services
rosserial_generate(rospack, dest, ROS_TO_EMBEDDED_TYPES)

