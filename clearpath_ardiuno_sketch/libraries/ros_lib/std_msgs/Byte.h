#ifndef _ROS_std_msgs_Byte_h
#define _ROS_std_msgs_Byte_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/byte.h"

namespace std_msgs
{

  class Byte : public ros::Msg
  {
    public:
      std_msgs::byte data;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "std_msgs/Byte"; };
    const char * getMD5(){ return "ad736a2e8818154c487bb80fe42ce43b"; };

  };

}
#endif