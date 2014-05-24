#ifndef _ROS_SERVICE_setInitialPose_h
#define _ROS_SERVICE_setInitialPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace state_estimation
{

static const char SETINITIALPOSE[] = "state_estimation/setInitialPose";

  class setInitialPoseRequest : public ros::Msg
  {
    public:
      int16_t a;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_a;
      u_a.real = this->a;
      *(outbuffer + offset + 0) = (u_a.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->a);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_a;
      u_a.base = 0;
      u_a.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->a = u_a.real;
      offset += sizeof(this->a);
     return offset;
    }

    const char * getType(){ return SETINITIALPOSE; };
    const char * getMD5(){ return "55dc7b156d5624062efec16350895ec2"; };

  };

  class setInitialPoseResponse : public ros::Msg
  {
    public:
      int16_t b;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_b;
      u_b.real = this->b;
      *(outbuffer + offset + 0) = (u_b.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_b.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->b);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_b;
      u_b.base = 0;
      u_b.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_b.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->b = u_b.real;
      offset += sizeof(this->b);
     return offset;
    }

    const char * getType(){ return SETINITIALPOSE; };
    const char * getMD5(){ return "dc6b6d40e1ef1c90bed188cd9ca36967"; };

  };

  class setInitialPose {
    public:
    typedef setInitialPoseRequest Request;
    typedef setInitialPoseResponse Response;
  };

}
#endif
