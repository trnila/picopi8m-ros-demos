#ifndef _ROS_SERVICE_Pause_h
#define _ROS_SERVICE_Pause_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace demo_fake_m4_sensor
{

static const char PAUSE[] = "demo_fake_m4_sensor/Pause";

  class PauseRequest : public ros::Msg
  {
    public:
      typedef bool _pause_type;
      _pause_type pause;

    PauseRequest():
      pause(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_pause;
      u_pause.real = this->pause;
      *(outbuffer + offset + 0) = (u_pause.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pause);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_pause;
      u_pause.base = 0;
      u_pause.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pause = u_pause.real;
      offset += sizeof(this->pause);
     return offset;
    }

    const char * getType(){ return PAUSE; };
    const char * getMD5(){ return "769db6046968e824fd55099ac609f204"; };

  };

  class PauseResponse : public ros::Msg
  {
    public:

    PauseResponse()
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

    const char * getType(){ return PAUSE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Pause {
    public:
    typedef PauseRequest Request;
    typedef PauseResponse Response;
  };

}
#endif
