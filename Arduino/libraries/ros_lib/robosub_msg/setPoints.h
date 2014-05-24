#ifndef _ROS_robosub_msg_setPoints_h
#define _ROS_robosub_msg_setPoints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "robosub_msg/ValueControl.h"

namespace robosub_msg
{

  class setPoints : public ros::Msg
  {
    public:
      robosub_msg::ValueControl XPos;
      robosub_msg::ValueControl YPos;
      robosub_msg::ValueControl Yaw;
      robosub_msg::ValueControl Pitch;
      robosub_msg::ValueControl XSpeed;
      robosub_msg::ValueControl YSpeed;
      robosub_msg::ValueControl YawSpeed;
      robosub_msg::ValueControl Depth;
      robosub_msg::ValueControl DepthSpeed;
      char * Frame;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->XPos.serialize(outbuffer + offset);
      offset += this->YPos.serialize(outbuffer + offset);
      offset += this->Yaw.serialize(outbuffer + offset);
      offset += this->Pitch.serialize(outbuffer + offset);
      offset += this->XSpeed.serialize(outbuffer + offset);
      offset += this->YSpeed.serialize(outbuffer + offset);
      offset += this->YawSpeed.serialize(outbuffer + offset);
      offset += this->Depth.serialize(outbuffer + offset);
      offset += this->DepthSpeed.serialize(outbuffer + offset);
      uint32_t length_Frame = strlen( (const char*) this->Frame);
      memcpy(outbuffer + offset, &length_Frame, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->Frame, length_Frame);
      offset += length_Frame;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->XPos.deserialize(inbuffer + offset);
      offset += this->YPos.deserialize(inbuffer + offset);
      offset += this->Yaw.deserialize(inbuffer + offset);
      offset += this->Pitch.deserialize(inbuffer + offset);
      offset += this->XSpeed.deserialize(inbuffer + offset);
      offset += this->YSpeed.deserialize(inbuffer + offset);
      offset += this->YawSpeed.deserialize(inbuffer + offset);
      offset += this->Depth.deserialize(inbuffer + offset);
      offset += this->DepthSpeed.deserialize(inbuffer + offset);
      uint32_t length_Frame;
      memcpy(&length_Frame, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_Frame; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_Frame-1]=0;
      this->Frame = (char *)(inbuffer + offset-1);
      offset += length_Frame;
     return offset;
    }

    const char * getType(){ return "robosub_msg/setPoints"; };
    const char * getMD5(){ return "1d45a1baf95e8b4c45c62d30c3e2357e"; };

  };

}
#endif