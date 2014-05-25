#ifndef _ROS_SERVICE_UpdateBatteryLights_h
#define _ROS_SERVICE_UpdateBatteryLights_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "blinky/RGB.h"

namespace blinky
{

static const char UPDATEBATTERYLIGHTS[] = "blinky/UpdateBatteryLights";

  class UpdateBatteryLightsRequest : public ros::Msg
  {
    public:
      uint8_t colors_length;
      blinky::RGB st_colors;
      blinky::RGB * colors;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = colors_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < colors_length; i++){
      offset += this->colors[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t colors_lengthT = *(inbuffer + offset++);
      if(colors_lengthT > colors_length)
        this->colors = (blinky::RGB*)realloc(this->colors, colors_lengthT * sizeof(blinky::RGB));
      offset += 3;
      colors_length = colors_lengthT;
      for( uint8_t i = 0; i < colors_length; i++){
      offset += this->st_colors.deserialize(inbuffer + offset);
        memcpy( &(this->colors[i]), &(this->st_colors), sizeof(blinky::RGB));
      }
     return offset;
    }

    const char * getType(){ return UPDATEBATTERYLIGHTS; };
    const char * getMD5(){ return "8823669ce5b9d7986742df5c9387b270"; };

  };

  class UpdateBatteryLightsResponse : public ros::Msg
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

    const char * getType(){ return UPDATEBATTERYLIGHTS; };
    const char * getMD5(){ return "0b13460cb14006d3852674b4c614f25f"; };

  };

  class UpdateBatteryLights {
    public:
    typedef UpdateBatteryLightsRequest Request;
    typedef UpdateBatteryLightsResponse Response;
  };

}
#endif
