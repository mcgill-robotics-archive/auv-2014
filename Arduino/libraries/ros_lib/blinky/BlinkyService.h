#ifndef _ROS_SERVICE_BlinkyService_h
#define _ROS_SERVICE_BlinkyService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "blinky/RGB.h"

namespace blinky
{

static const char BLINKYSERVICE[] = "blinky/BlinkyService";

  class BlinkyServiceRequest : public ros::Msg
  {
    public:
      uint8_t btColors_length;
      blinky::RGB st_btColors;
      blinky::RGB * btColors;
      int8_t blinkyID;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = btColors_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < btColors_length; i++){
      offset += this->btColors[i].serialize(outbuffer + offset);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_blinkyID;
      u_blinkyID.real = this->blinkyID;
      *(outbuffer + offset + 0) = (u_blinkyID.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blinkyID);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t btColors_lengthT = *(inbuffer + offset++);
      if(btColors_lengthT > btColors_length)
        this->btColors = (blinky::RGB*)realloc(this->btColors, btColors_lengthT * sizeof(blinky::RGB));
      offset += 3;
      btColors_length = btColors_lengthT;
      for( uint8_t i = 0; i < btColors_length; i++){
      offset += this->st_btColors.deserialize(inbuffer + offset);
        memcpy( &(this->btColors[i]), &(this->st_btColors), sizeof(blinky::RGB));
      }
      union {
        int8_t real;
        uint8_t base;
      } u_blinkyID;
      u_blinkyID.base = 0;
      u_blinkyID.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blinkyID = u_blinkyID.real;
      offset += sizeof(this->blinkyID);
     return offset;
    }

    const char * getType(){ return BLINKYSERVICE; };
    const char * getMD5(){ return "d0cd46e4e7ed256e27ee8738877fab35"; };

  };

  class BlinkyServiceResponse : public ros::Msg
  {
    public:
      int8_t success;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    const char * getType(){ return BLINKYSERVICE; };
    const char * getMD5(){ return "0b13460cb14006d3852674b4c614f25f"; };

  };

  class BlinkyService {
    public:
    typedef BlinkyServiceRequest Request;
    typedef BlinkyServiceResponse Response;
  };

}
#endif
