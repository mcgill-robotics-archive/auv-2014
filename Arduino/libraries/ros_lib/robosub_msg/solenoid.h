#ifndef _ROS_robosub_msg_solenoid_h
#define _ROS_robosub_msg_solenoid_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Bool.h"

namespace robosub_msg
{

  class solenoid : public ros::Msg
  {
    public:
      std_msgs::Bool torpedo1;
      std_msgs::Bool torpedo2;
      std_msgs::Bool dropper1;
      std_msgs::Bool dropper2;
      std_msgs::Bool grabber1;
      std_msgs::Bool grabber2;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->torpedo1.serialize(outbuffer + offset);
      offset += this->torpedo2.serialize(outbuffer + offset);
      offset += this->dropper1.serialize(outbuffer + offset);
      offset += this->dropper2.serialize(outbuffer + offset);
      offset += this->grabber1.serialize(outbuffer + offset);
      offset += this->grabber2.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->torpedo1.deserialize(inbuffer + offset);
      offset += this->torpedo2.deserialize(inbuffer + offset);
      offset += this->dropper1.deserialize(inbuffer + offset);
      offset += this->dropper2.deserialize(inbuffer + offset);
      offset += this->grabber1.deserialize(inbuffer + offset);
      offset += this->grabber2.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "robosub_msg/solenoid"; };
    const char * getMD5(){ return "7049a6757f9d71e7c5c744e407a6083a"; };

  };

}
#endif