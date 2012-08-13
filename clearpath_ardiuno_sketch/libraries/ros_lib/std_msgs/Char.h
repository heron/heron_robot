#ifndef _ROS_std_msgs_Char_h
#define _ROS_std_msgs_Char_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/char.h"

namespace std_msgs
{

  class Char : public ros::Msg
  {
    public:
      std_msgs::char data;

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

    const char * getType(){ return "std_msgs/Char"; };
    const char * getMD5(){ return "1bf77f25acecdedba0e224b162199717"; };

  };

}
#endif