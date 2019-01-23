#ifndef _ROS_SERVICE_Stop_h
#define _ROS_SERVICE_Stop_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace m4ctrl
{

static const char STOP[] = "m4ctrl/Stop";

  class StopRequest : public ros::Msg
  {
    public:

    StopRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return STOP; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class StopResponse : public ros::Msg
  {
    public:

    StopResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return STOP; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Stop {
    public:
    typedef StopRequest Request;
    typedef StopResponse Response;
  };

}
#endif
