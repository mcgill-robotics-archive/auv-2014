#ifndef _ROS_planner_setPoints_h
#define _ROS_planner_setPoints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "planner/ValueControl.h"

namespace planner
{

  class setPoints : public ros::Msg
  {
    public:
      planner::ValueControl XPos;
      planner::ValueControl YPos;
      planner::ValueControl Yaw;
      planner::ValueControl Pitch;
      planner::ValueControl XSpeed;
      planner::ValueControl YSpeed;
      planner::ValueControl YawSpeed;
      planner::ValueControl Depth;

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
     return offset;
    }

    const char * getType(){ return "planner/setPoints"; };
    const char * getMD5(){ return "61330c643c485a1d6154db4f43c34209"; };

  };

}
#endif