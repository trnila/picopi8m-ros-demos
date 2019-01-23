#ifndef _ROS_SERVICE_Start_h
#define _ROS_SERVICE_Start_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace m4ctrl
{

static const char START[] = "m4ctrl/Start";

  class StartRequest : public ros::Msg
  {
    public:
      typedef const char* _firmware_path_type;
      _firmware_path_type firmware_path;

    StartRequest():
      firmware_path("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_firmware_path = strlen(this->firmware_path);
      varToArr(outbuffer + offset, length_firmware_path);
      offset += 4;
      memcpy(outbuffer + offset, this->firmware_path, length_firmware_path);
      offset += length_firmware_path;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_firmware_path;
      arrToVar(length_firmware_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_firmware_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_firmware_path-1]=0;
      this->firmware_path = (char *)(inbuffer + offset-1);
      offset += length_firmware_path;
     return offset;
    }

    const char * getType(){ return START; };
    const char * getMD5(){ return "9a0573be22b698fee7a2469ff138061a"; };

  };

  class StartResponse : public ros::Msg
  {
    public:

    StartResponse()
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

    const char * getType(){ return START; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Start {
    public:
    typedef StartRequest Request;
    typedef StartResponse Response;
  };

}
#endif
