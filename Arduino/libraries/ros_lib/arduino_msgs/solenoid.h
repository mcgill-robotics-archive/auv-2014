#ifndef _ROS_arduino_msgs_solenoid_h
#define _ROS_arduino_msgs_solenoid_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Bool.h"

namespace arduino_msgs
{

  class solenoid : public ros::Msg
  {
    public:
      std_msgs::Bool torpedo0;
      std_msgs::Bool torpedo1;
      std_msgs::Bool dropper0;
      std_msgs::Bool dropper1;
      std_msgs::Bool grabber0;
      std_msgs::Bool grabber1;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->torpedo0.serialize(outbuffer + offset);
      offset += this->torpedo1.serialize(outbuffer + offset);
      offset += this->dropper0.serialize(outbuffer + offset);
      offset += this->dropper1.serialize(outbuffer + offset);
      offset += this->grabber0.serialize(outbuffer + offset);
      offset += this->grabber1.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->torpedo0.deserialize(inbuffer + offset);
      offset += this->torpedo1.deserialize(inbuffer + offset);
      offset += this->dropper0.deserialize(inbuffer + offset);
      offset += this->dropper1.deserialize(inbuffer + offset);
      offset += this->grabber0.deserialize(inbuffer + offset);
      offset += this->grabber1.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "arduino_msgs/solenoid"; };
    const char * getMD5(){ return "b463c3b869441179ff39cfca6b77edf0"; };

  };

}
#endif