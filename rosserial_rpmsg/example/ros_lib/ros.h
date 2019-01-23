#pragma once
#include "ros/node_handle.h"
#include "RpmsgHardware.h"

namespace ros {
  typedef NodeHandle_<RpmsgHardware> NodeHandle;
}
