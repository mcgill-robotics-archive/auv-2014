#ifndef _ROS_hydrophones_tdoa_h
#define _ROS_hydrophones_tdoa_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hydrophones
{

  class tdoa : public ros::Msg
  {
    public:
      float tdoa_1;
      float tdoa_2;
      float tdoa_3;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_tdoa_1;
      u_tdoa_1.real = this->tdoa_1;
      *(outbuffer + offset + 0) = (u_tdoa_1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tdoa_1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tdoa_1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tdoa_1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tdoa_1);
      union {
        float real;
        uint32_t base;
      } u_tdoa_2;
      u_tdoa_2.real = this->tdoa_2;
      *(outbuffer + offset + 0) = (u_tdoa_2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tdoa_2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tdoa_2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tdoa_2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tdoa_2);
      union {
        float real;
        uint32_t base;
      } u_tdoa_3;
      u_tdoa_3.real = this->tdoa_3;
      *(outbuffer + offset + 0) = (u_tdoa_3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tdoa_3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tdoa_3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tdoa_3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tdoa_3);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_tdoa_1;
      u_tdoa_1.base = 0;
      u_tdoa_1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tdoa_1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tdoa_1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tdoa_1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tdoa_1 = u_tdoa_1.real;
      offset += sizeof(this->tdoa_1);
      union {
        float real;
        uint32_t base;
      } u_tdoa_2;
      u_tdoa_2.base = 0;
      u_tdoa_2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tdoa_2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tdoa_2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tdoa_2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tdoa_2 = u_tdoa_2.real;
      offset += sizeof(this->tdoa_2);
      union {
        float real;
        uint32_t base;
      } u_tdoa_3;
      u_tdoa_3.base = 0;
      u_tdoa_3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tdoa_3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tdoa_3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tdoa_3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tdoa_3 = u_tdoa_3.real;
      offset += sizeof(this->tdoa_3);
     return offset;
    }

    const char * getType(){ return "hydrophones/tdoa"; };
    const char * getMD5(){ return "ae91add6a824b87d469e9a2d472aeb25"; };

  };

}
#endif