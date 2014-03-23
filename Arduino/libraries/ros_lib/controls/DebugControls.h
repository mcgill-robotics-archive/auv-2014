#ifndef _ROS_controls_DebugControls_h
#define _ROS_controls_DebugControls_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "controls/VectorPID.h"

namespace controls
{

  class DebugControls : public ros::Msg
  {
    public:
      controls::VectorPID xError;
      controls::VectorPID yError;
      controls::VectorPID depthError;
      controls::VectorPID pitchError;
      controls::VectorPID yawError;
      controls::VectorPID xGain;
      controls::VectorPID yGain;
      controls::VectorPID depthGain;
      controls::VectorPID pitchGain;
      controls::VectorPID yawGain;
      controls::VectorPID xForce;
      controls::VectorPID yForce;
      controls::VectorPID depthForce;
      controls::VectorPID pitchForce;
      controls::VectorPID yawForce;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->xError.serialize(outbuffer + offset);
      offset += this->yError.serialize(outbuffer + offset);
      offset += this->depthError.serialize(outbuffer + offset);
      offset += this->pitchError.serialize(outbuffer + offset);
      offset += this->yawError.serialize(outbuffer + offset);
      offset += this->xGain.serialize(outbuffer + offset);
      offset += this->yGain.serialize(outbuffer + offset);
      offset += this->depthGain.serialize(outbuffer + offset);
      offset += this->pitchGain.serialize(outbuffer + offset);
      offset += this->yawGain.serialize(outbuffer + offset);
      offset += this->xForce.serialize(outbuffer + offset);
      offset += this->yForce.serialize(outbuffer + offset);
      offset += this->depthForce.serialize(outbuffer + offset);
      offset += this->pitchForce.serialize(outbuffer + offset);
      offset += this->yawForce.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->xError.deserialize(inbuffer + offset);
      offset += this->yError.deserialize(inbuffer + offset);
      offset += this->depthError.deserialize(inbuffer + offset);
      offset += this->pitchError.deserialize(inbuffer + offset);
      offset += this->yawError.deserialize(inbuffer + offset);
      offset += this->xGain.deserialize(inbuffer + offset);
      offset += this->yGain.deserialize(inbuffer + offset);
      offset += this->depthGain.deserialize(inbuffer + offset);
      offset += this->pitchGain.deserialize(inbuffer + offset);
      offset += this->yawGain.deserialize(inbuffer + offset);
      offset += this->xForce.deserialize(inbuffer + offset);
      offset += this->yForce.deserialize(inbuffer + offset);
      offset += this->depthForce.deserialize(inbuffer + offset);
      offset += this->pitchForce.deserialize(inbuffer + offset);
      offset += this->yawForce.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "controls/DebugControls"; };
    const char * getMD5(){ return "2c47c70851217027532bd882f9fcb36e"; };

  };

}
#endif