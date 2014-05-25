#ifndef _ROS_hydrophones_channels_h
#define _ROS_hydrophones_channels_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hydrophones
{

  class channels : public ros::Msg
  {
    public:
      uint8_t channel_0_length;
      float st_channel_0;
      float * channel_0;
      uint8_t channel_1_length;
      float st_channel_1;
      float * channel_1;
      uint8_t channel_2_length;
      float st_channel_2;
      float * channel_2;
      uint8_t channel_3_length;
      float st_channel_3;
      float * channel_3;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = channel_0_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channel_0_length; i++){
      union {
        float real;
        uint32_t base;
      } u_channel_0i;
      u_channel_0i.real = this->channel_0[i];
      *(outbuffer + offset + 0) = (u_channel_0i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_channel_0i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_channel_0i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_channel_0i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->channel_0[i]);
      }
      *(outbuffer + offset++) = channel_1_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channel_1_length; i++){
      union {
        float real;
        uint32_t base;
      } u_channel_1i;
      u_channel_1i.real = this->channel_1[i];
      *(outbuffer + offset + 0) = (u_channel_1i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_channel_1i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_channel_1i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_channel_1i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->channel_1[i]);
      }
      *(outbuffer + offset++) = channel_2_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channel_2_length; i++){
      union {
        float real;
        uint32_t base;
      } u_channel_2i;
      u_channel_2i.real = this->channel_2[i];
      *(outbuffer + offset + 0) = (u_channel_2i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_channel_2i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_channel_2i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_channel_2i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->channel_2[i]);
      }
      *(outbuffer + offset++) = channel_3_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channel_3_length; i++){
      union {
        float real;
        uint32_t base;
      } u_channel_3i;
      u_channel_3i.real = this->channel_3[i];
      *(outbuffer + offset + 0) = (u_channel_3i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_channel_3i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_channel_3i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_channel_3i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->channel_3[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t channel_0_lengthT = *(inbuffer + offset++);
      if(channel_0_lengthT > channel_0_length)
        this->channel_0 = (float*)realloc(this->channel_0, channel_0_lengthT * sizeof(float));
      offset += 3;
      channel_0_length = channel_0_lengthT;
      for( uint8_t i = 0; i < channel_0_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_channel_0;
      u_st_channel_0.base = 0;
      u_st_channel_0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_channel_0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_channel_0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_channel_0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_channel_0 = u_st_channel_0.real;
      offset += sizeof(this->st_channel_0);
        memcpy( &(this->channel_0[i]), &(this->st_channel_0), sizeof(float));
      }
      uint8_t channel_1_lengthT = *(inbuffer + offset++);
      if(channel_1_lengthT > channel_1_length)
        this->channel_1 = (float*)realloc(this->channel_1, channel_1_lengthT * sizeof(float));
      offset += 3;
      channel_1_length = channel_1_lengthT;
      for( uint8_t i = 0; i < channel_1_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_channel_1;
      u_st_channel_1.base = 0;
      u_st_channel_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_channel_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_channel_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_channel_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_channel_1 = u_st_channel_1.real;
      offset += sizeof(this->st_channel_1);
        memcpy( &(this->channel_1[i]), &(this->st_channel_1), sizeof(float));
      }
      uint8_t channel_2_lengthT = *(inbuffer + offset++);
      if(channel_2_lengthT > channel_2_length)
        this->channel_2 = (float*)realloc(this->channel_2, channel_2_lengthT * sizeof(float));
      offset += 3;
      channel_2_length = channel_2_lengthT;
      for( uint8_t i = 0; i < channel_2_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_channel_2;
      u_st_channel_2.base = 0;
      u_st_channel_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_channel_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_channel_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_channel_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_channel_2 = u_st_channel_2.real;
      offset += sizeof(this->st_channel_2);
        memcpy( &(this->channel_2[i]), &(this->st_channel_2), sizeof(float));
      }
      uint8_t channel_3_lengthT = *(inbuffer + offset++);
      if(channel_3_lengthT > channel_3_length)
        this->channel_3 = (float*)realloc(this->channel_3, channel_3_lengthT * sizeof(float));
      offset += 3;
      channel_3_length = channel_3_lengthT;
      for( uint8_t i = 0; i < channel_3_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_channel_3;
      u_st_channel_3.base = 0;
      u_st_channel_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_channel_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_channel_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_channel_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_channel_3 = u_st_channel_3.real;
      offset += sizeof(this->st_channel_3);
        memcpy( &(this->channel_3[i]), &(this->st_channel_3), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "hydrophones/channels"; };
    const char * getMD5(){ return "9ce473e48f4b0055d40a7f4dbb9df095"; };

  };

}
#endif