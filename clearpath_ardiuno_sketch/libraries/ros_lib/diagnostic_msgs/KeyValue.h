#ifndef _ROS_diagnostic_msgs_KeyValue_h
#define _ROS_diagnostic_msgs_KeyValue_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace diagnostic_msgs
{

  class KeyValue : public ros::Msg
  {
    public:
      char * key;
      char * value;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t * length_key = (uint32_t *)(outbuffer + offset);
      *length_key = strlen( (const char*) this->key);
      offset += 4;
      memcpy(outbuffer + offset, this->key, *length_key);
      offset += *length_key;
      uint32_t * length_value = (uint32_t *)(outbuffer + offset);
      *length_value = strlen( (const char*) this->value);
      offset += 4;
      memcpy(outbuffer + offset, this->value, *length_value);
      offset += *length_value;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_key = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_key; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_key-1]=0;
      this->key = (char *)(inbuffer + offset-1);
      offset += length_key;
      uint32_t length_value = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_value; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_value-1]=0;
      this->value = (char *)(inbuffer + offset-1);
      offset += length_value;
     return offset;
    }

    const char * getType(){ return "diagnostic_msgs/KeyValue"; };
    const char * getMD5(){ return "cf57fdc6617a881a88c16e768132149c"; };

  };

}
#endif