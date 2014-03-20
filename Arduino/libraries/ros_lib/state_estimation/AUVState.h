#ifndef _ROS_state_estimation_AUVState_h
#define _ROS_state_estimation_AUVState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "computer_vision/VisibleObjectData.h"
#include "geometry_msgs/Vector3.h"

namespace state_estimation
{

  class AUVState : public ros::Msg
  {
    public:
      computer_vision::VisibleObjectData visibleObjectData;
      bool hasTarget;
      float depth;
      float pitch;
      float pitch_rate;
      float roll;
      float roll_rate;
      geometry_msgs::Vector3 velocity;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->visibleObjectData.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_hasTarget;
      u_hasTarget.real = this->hasTarget;
      *(outbuffer + offset + 0) = (u_hasTarget.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->hasTarget);
      int32_t * val_depth = (int32_t *) &(this->depth);
      int32_t exp_depth = (((*val_depth)>>23)&255);
      if(exp_depth != 0)
        exp_depth += 1023-127;
      int32_t sig_depth = *val_depth;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_depth<<5) & 0xff;
      *(outbuffer + offset++) = (sig_depth>>3) & 0xff;
      *(outbuffer + offset++) = (sig_depth>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_depth<<4) & 0xF0) | ((sig_depth>>19)&0x0F);
      *(outbuffer + offset++) = (exp_depth>>4) & 0x7F;
      if(this->depth < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_pitch = (int32_t *) &(this->pitch);
      int32_t exp_pitch = (((*val_pitch)>>23)&255);
      if(exp_pitch != 0)
        exp_pitch += 1023-127;
      int32_t sig_pitch = *val_pitch;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_pitch<<5) & 0xff;
      *(outbuffer + offset++) = (sig_pitch>>3) & 0xff;
      *(outbuffer + offset++) = (sig_pitch>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_pitch<<4) & 0xF0) | ((sig_pitch>>19)&0x0F);
      *(outbuffer + offset++) = (exp_pitch>>4) & 0x7F;
      if(this->pitch < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_pitch_rate = (int32_t *) &(this->pitch_rate);
      int32_t exp_pitch_rate = (((*val_pitch_rate)>>23)&255);
      if(exp_pitch_rate != 0)
        exp_pitch_rate += 1023-127;
      int32_t sig_pitch_rate = *val_pitch_rate;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_pitch_rate<<5) & 0xff;
      *(outbuffer + offset++) = (sig_pitch_rate>>3) & 0xff;
      *(outbuffer + offset++) = (sig_pitch_rate>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_pitch_rate<<4) & 0xF0) | ((sig_pitch_rate>>19)&0x0F);
      *(outbuffer + offset++) = (exp_pitch_rate>>4) & 0x7F;
      if(this->pitch_rate < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_roll = (int32_t *) &(this->roll);
      int32_t exp_roll = (((*val_roll)>>23)&255);
      if(exp_roll != 0)
        exp_roll += 1023-127;
      int32_t sig_roll = *val_roll;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_roll<<5) & 0xff;
      *(outbuffer + offset++) = (sig_roll>>3) & 0xff;
      *(outbuffer + offset++) = (sig_roll>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_roll<<4) & 0xF0) | ((sig_roll>>19)&0x0F);
      *(outbuffer + offset++) = (exp_roll>>4) & 0x7F;
      if(this->roll < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_roll_rate = (int32_t *) &(this->roll_rate);
      int32_t exp_roll_rate = (((*val_roll_rate)>>23)&255);
      if(exp_roll_rate != 0)
        exp_roll_rate += 1023-127;
      int32_t sig_roll_rate = *val_roll_rate;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_roll_rate<<5) & 0xff;
      *(outbuffer + offset++) = (sig_roll_rate>>3) & 0xff;
      *(outbuffer + offset++) = (sig_roll_rate>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_roll_rate<<4) & 0xF0) | ((sig_roll_rate>>19)&0x0F);
      *(outbuffer + offset++) = (exp_roll_rate>>4) & 0x7F;
      if(this->roll_rate < 0) *(outbuffer + offset -1) |= 0x80;
      offset += this->velocity.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->visibleObjectData.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_hasTarget;
      u_hasTarget.base = 0;
      u_hasTarget.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hasTarget = u_hasTarget.real;
      offset += sizeof(this->hasTarget);
      uint32_t * val_depth = (uint32_t*) &(this->depth);
      offset += 3;
      *val_depth = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_depth |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_depth |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_depth |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_depth = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_depth |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_depth !=0)
        *val_depth |= ((exp_depth)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->depth = -this->depth;
      uint32_t * val_pitch = (uint32_t*) &(this->pitch);
      offset += 3;
      *val_pitch = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_pitch |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_pitch |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_pitch |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_pitch = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_pitch |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_pitch !=0)
        *val_pitch |= ((exp_pitch)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->pitch = -this->pitch;
      uint32_t * val_pitch_rate = (uint32_t*) &(this->pitch_rate);
      offset += 3;
      *val_pitch_rate = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_pitch_rate |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_pitch_rate |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_pitch_rate |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_pitch_rate = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_pitch_rate |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_pitch_rate !=0)
        *val_pitch_rate |= ((exp_pitch_rate)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->pitch_rate = -this->pitch_rate;
      uint32_t * val_roll = (uint32_t*) &(this->roll);
      offset += 3;
      *val_roll = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_roll |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_roll |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_roll |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_roll = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_roll |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_roll !=0)
        *val_roll |= ((exp_roll)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->roll = -this->roll;
      uint32_t * val_roll_rate = (uint32_t*) &(this->roll_rate);
      offset += 3;
      *val_roll_rate = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_roll_rate |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_roll_rate |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_roll_rate |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_roll_rate = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_roll_rate |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_roll_rate !=0)
        *val_roll_rate |= ((exp_roll_rate)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->roll_rate = -this->roll_rate;
      offset += this->velocity.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "state_estimation/AUVState"; };
    const char * getMD5(){ return "f8e31d30fe47e2a47f58d12d279799fc"; };

  };

}
#endif