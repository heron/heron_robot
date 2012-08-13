#ifndef _ROS_SERVICE_SelfTest_h
#define _ROS_SERVICE_SelfTest_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/byte.h"

namespace diagnostic_msgs
{

static const char SELFTEST[] = "diagnostic_msgs/SelfTest";

  class SelfTestRequest : public ros::Msg
  {
    public:

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

    const char * getType(){ return SELFTEST; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SelfTestResponse : public ros::Msg
  {
    public:
      char * id;
      diagnostic_msgs::byte passed;
      uint8_t status_length;
      diagnostic_msgs::DiagnosticStatus st_status;
      diagnostic_msgs::DiagnosticStatus * status;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t * length_id = (uint32_t *)(outbuffer + offset);
      *length_id = strlen( (const char*) this->id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, *length_id);
      offset += *length_id;
      offset += this->passed.serialize(outbuffer + offset);
      *(outbuffer + offset++) = status_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < status_length; i++){
      offset += this->status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_id = *(uint32_t *)(inbuffer + offset);
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
      offset += this->passed.deserialize(inbuffer + offset);
      uint8_t status_lengthT = *(inbuffer + offset++);
      if(status_lengthT > status_length)
        this->status = (diagnostic_msgs::DiagnosticStatus*)realloc(this->status, status_lengthT * sizeof(diagnostic_msgs::DiagnosticStatus));
      offset += 3;
      status_length = status_lengthT;
      for( uint8_t i = 0; i < status_length; i++){
      offset += this->st_status.deserialize(inbuffer + offset);
        memcpy( &(this->status[i]), &(this->st_status), sizeof(diagnostic_msgs::DiagnosticStatus));
      }
     return offset;
    }

    const char * getType(){ return SELFTEST; };
    const char * getMD5(){ return "74c9372c870a76da4fc2b3973978b898"; };

  };

  class SelfTest {
    public:
    typedef SelfTestRequest Request;
    typedef SelfTestResponse Response;
  };

}
#endif