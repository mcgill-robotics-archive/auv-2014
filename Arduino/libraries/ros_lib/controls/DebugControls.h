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
      int32_t thrust_surge_port;
      int32_t thrust_surge_starboard;
      int32_t thrust_sway_bow;
      int32_t thrust_sway_stern;
      int32_t thrust_heave_bow;
      int32_t thrust_heave_stern;

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
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_surge_port;
      u_thrust_surge_port.real = this->thrust_surge_port;
      *(outbuffer + offset + 0) = (u_thrust_surge_port.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrust_surge_port.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_thrust_surge_port.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_thrust_surge_port.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thrust_surge_port);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_surge_starboard;
      u_thrust_surge_starboard.real = this->thrust_surge_starboard;
      *(outbuffer + offset + 0) = (u_thrust_surge_starboard.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrust_surge_starboard.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_thrust_surge_starboard.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_thrust_surge_starboard.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thrust_surge_starboard);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_sway_bow;
      u_thrust_sway_bow.real = this->thrust_sway_bow;
      *(outbuffer + offset + 0) = (u_thrust_sway_bow.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrust_sway_bow.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_thrust_sway_bow.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_thrust_sway_bow.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thrust_sway_bow);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_sway_stern;
      u_thrust_sway_stern.real = this->thrust_sway_stern;
      *(outbuffer + offset + 0) = (u_thrust_sway_stern.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrust_sway_stern.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_thrust_sway_stern.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_thrust_sway_stern.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thrust_sway_stern);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_heave_bow;
      u_thrust_heave_bow.real = this->thrust_heave_bow;
      *(outbuffer + offset + 0) = (u_thrust_heave_bow.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrust_heave_bow.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_thrust_heave_bow.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_thrust_heave_bow.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thrust_heave_bow);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_heave_stern;
      u_thrust_heave_stern.real = this->thrust_heave_stern;
      *(outbuffer + offset + 0) = (u_thrust_heave_stern.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_thrust_heave_stern.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_thrust_heave_stern.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_thrust_heave_stern.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->thrust_heave_stern);
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
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_surge_port;
      u_thrust_surge_port.base = 0;
      u_thrust_surge_port.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thrust_surge_port.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_thrust_surge_port.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_thrust_surge_port.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->thrust_surge_port = u_thrust_surge_port.real;
      offset += sizeof(this->thrust_surge_port);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_surge_starboard;
      u_thrust_surge_starboard.base = 0;
      u_thrust_surge_starboard.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thrust_surge_starboard.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_thrust_surge_starboard.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_thrust_surge_starboard.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->thrust_surge_starboard = u_thrust_surge_starboard.real;
      offset += sizeof(this->thrust_surge_starboard);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_sway_bow;
      u_thrust_sway_bow.base = 0;
      u_thrust_sway_bow.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thrust_sway_bow.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_thrust_sway_bow.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_thrust_sway_bow.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->thrust_sway_bow = u_thrust_sway_bow.real;
      offset += sizeof(this->thrust_sway_bow);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_sway_stern;
      u_thrust_sway_stern.base = 0;
      u_thrust_sway_stern.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thrust_sway_stern.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_thrust_sway_stern.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_thrust_sway_stern.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->thrust_sway_stern = u_thrust_sway_stern.real;
      offset += sizeof(this->thrust_sway_stern);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_heave_bow;
      u_thrust_heave_bow.base = 0;
      u_thrust_heave_bow.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thrust_heave_bow.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_thrust_heave_bow.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_thrust_heave_bow.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->thrust_heave_bow = u_thrust_heave_bow.real;
      offset += sizeof(this->thrust_heave_bow);
      union {
        int32_t real;
        uint32_t base;
      } u_thrust_heave_stern;
      u_thrust_heave_stern.base = 0;
      u_thrust_heave_stern.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_thrust_heave_stern.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_thrust_heave_stern.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_thrust_heave_stern.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->thrust_heave_stern = u_thrust_heave_stern.real;
      offset += sizeof(this->thrust_heave_stern);
     return offset;
    }

    const char * getType(){ return "controls/DebugControls"; };
    const char * getMD5(){ return "07d63863f19e5d52d3eef13f925e3b65"; };

  };

}
#endif