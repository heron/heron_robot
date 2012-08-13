#ifndef _ROS_rosserial_msgs_Log_h
#define _ROS_rosserial_msgs_Log_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosserial_msgs
{

  class Log : public ros::Msg
  {
    public:
      uint8_t level;
      char * msg;
      enum { DEBUG = 0 };
      enum { INFO = 1 };
      enum { WARN = 2 };
      enum { ERROR = 3 };
      enum { FATAL = 4 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->level >> (8 * 0)) & 0xFF;
      offset += sizeof(this->level);
      uint32_t * length_msg = (uint32_t *)(outbuffer + offset);
      *length_msg = strlen( (const char*) this->msg);
      offset += 4;
      memcpy(outbuffer + offset, this->msg, *length_msg);
      offset += *length_msg;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->level |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      offset += sizeof(this->level);
      uint32_t length_msg = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_msg; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_msg-1]=0;
      this->msg = (char *)(inbuffer + offset-1);
      offset += length_msg;
     return offset;
    }

    const char * getType(){ return "rosserial_msgs/Log"; };
    const char * getMD5(){ return "7170d5aec999754ba0d9f762bf49b913"; };

  };

}
#endif