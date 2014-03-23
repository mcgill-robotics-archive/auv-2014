#ifndef _ROS_planner_ValueControl_h
#define _ROS_planner_ValueControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace planner
{

  class ValueControl : public ros::Msg
  {
    public:
      int8_t isActive;
      float data;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_isActive;
      u_isActive.real = this->isActive;
      *(outbuffer + offset + 0) = (u_isActive.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isActive);
      int32_t * val_data = (int32_t *) &(this->data);
      int32_t exp_data = (((*val_data)>>23)&255);
      if(exp_data != 0)
        exp_data += 1023-127;
      int32_t sig_data = *val_data;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_data<<5) & 0xff;
      *(outbuffer + offset++) = (sig_data>>3) & 0xff;
      *(outbuffer + offset++) = (sig_data>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_data<<4) & 0xF0) | ((sig_data>>19)&0x0F);
      *(outbuffer + offset++) = (exp_data>>4) & 0x7F;
      if(this->data < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_isActive;
      u_isActive.base = 0;
      u_isActive.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isActive = u_isActive.real;
      offset += sizeof(this->isActive);
      uint32_t * val_data = (uint32_t*) &(this->data);
      offset += 3;
      *val_data = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_data |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_data |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_data |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_data = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_data |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_data !=0)
        *val_data |= ((exp_data)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->data = -this->data;
     return offset;
    }

    const char * getType(){ return "planner/ValueControl"; };
    const char * getMD5(){ return "f1008d73418aa9a0553f339b2fc4c189"; };

  };

}
#endif