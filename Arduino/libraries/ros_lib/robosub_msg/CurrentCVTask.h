#ifndef _ROS_robosub_msg_CurrentCVTask_h
#define _ROS_robosub_msg_CurrentCVTask_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robosub_msg
{

  class CurrentCVTask : public ros::Msg
  {
    public:
      uint8_t currentCVTask;
      enum { NOTHING = 0 };
      enum { GATE = 1 };
      enum { BUOY = 2 };
      enum { LANE = 3 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->currentCVTask >> (8 * 0)) & 0xFF;
      offset += sizeof(this->currentCVTask);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->currentCVTask =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->currentCVTask);
     return offset;
    }

    const char * getType(){ return "robosub_msg/CurrentCVTask"; };
    const char * getMD5(){ return "726aab749cdee4d65593abb754b07e92"; };

  };

}
#endif