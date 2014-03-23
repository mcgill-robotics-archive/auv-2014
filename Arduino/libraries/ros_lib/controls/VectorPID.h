#ifndef _ROS_controls_VectorPID_h
#define _ROS_controls_VectorPID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace controls
{

  class VectorPID : public ros::Msg
  {
    public:
      float proportional;
      float integral;
      float derivative;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      int32_t * val_proportional = (int32_t *) &(this->proportional);
      int32_t exp_proportional = (((*val_proportional)>>23)&255);
      if(exp_proportional != 0)
        exp_proportional += 1023-127;
      int32_t sig_proportional = *val_proportional;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_proportional<<5) & 0xff;
      *(outbuffer + offset++) = (sig_proportional>>3) & 0xff;
      *(outbuffer + offset++) = (sig_proportional>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_proportional<<4) & 0xF0) | ((sig_proportional>>19)&0x0F);
      *(outbuffer + offset++) = (exp_proportional>>4) & 0x7F;
      if(this->proportional < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_integral = (int32_t *) &(this->integral);
      int32_t exp_integral = (((*val_integral)>>23)&255);
      if(exp_integral != 0)
        exp_integral += 1023-127;
      int32_t sig_integral = *val_integral;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_integral<<5) & 0xff;
      *(outbuffer + offset++) = (sig_integral>>3) & 0xff;
      *(outbuffer + offset++) = (sig_integral>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_integral<<4) & 0xF0) | ((sig_integral>>19)&0x0F);
      *(outbuffer + offset++) = (exp_integral>>4) & 0x7F;
      if(this->integral < 0) *(outbuffer + offset -1) |= 0x80;
      int32_t * val_derivative = (int32_t *) &(this->derivative);
      int32_t exp_derivative = (((*val_derivative)>>23)&255);
      if(exp_derivative != 0)
        exp_derivative += 1023-127;
      int32_t sig_derivative = *val_derivative;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_derivative<<5) & 0xff;
      *(outbuffer + offset++) = (sig_derivative>>3) & 0xff;
      *(outbuffer + offset++) = (sig_derivative>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_derivative<<4) & 0xF0) | ((sig_derivative>>19)&0x0F);
      *(outbuffer + offset++) = (exp_derivative>>4) & 0x7F;
      if(this->derivative < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t * val_proportional = (uint32_t*) &(this->proportional);
      offset += 3;
      *val_proportional = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_proportional |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_proportional |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_proportional |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_proportional = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_proportional |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_proportional !=0)
        *val_proportional |= ((exp_proportional)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->proportional = -this->proportional;
      uint32_t * val_integral = (uint32_t*) &(this->integral);
      offset += 3;
      *val_integral = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_integral |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_integral |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_integral |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_integral = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_integral |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_integral !=0)
        *val_integral |= ((exp_integral)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->integral = -this->integral;
      uint32_t * val_derivative = (uint32_t*) &(this->derivative);
      offset += 3;
      *val_derivative = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_derivative |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_derivative |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_derivative |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_derivative = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_derivative |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_derivative !=0)
        *val_derivative |= ((exp_derivative)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->derivative = -this->derivative;
     return offset;
    }

    const char * getType(){ return "controls/VectorPID"; };
    const char * getMD5(){ return "194e1ca0c1daaea2a375ddd8ccd75cb2"; };

  };

}
#endif