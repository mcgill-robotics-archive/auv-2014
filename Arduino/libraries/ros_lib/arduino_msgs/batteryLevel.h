#ifndef _ROS_arduino_msgs_batteryLevel_h
#define _ROS_arduino_msgs_batteryLevel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

  class batteryLevel : public ros::Msg
  {
    public:
      int8_t battery0;
      int8_t battery1;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_battery0;
      u_battery0.real = this->battery0;
      *(outbuffer + offset + 0) = (u_battery0.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery0);
      union {
        int8_t real;
        uint8_t base;
      } u_battery1;
      u_battery1.real = this->battery1;
      *(outbuffer + offset + 0) = (u_battery1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->battery1);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_battery0;
      u_battery0.base = 0;
      u_battery0.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->battery0 = u_battery0.real;
      offset += sizeof(this->battery0);
      union {
        int8_t real;
        uint8_t base;
      } u_battery1;
      u_battery1.base = 0;
      u_battery1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->battery1 = u_battery1.real;
      offset += sizeof(this->battery1);
     return offset;
    }

    const char * getType(){ return "arduino_msgs/batteryLevel"; };
    const char * getMD5(){ return "605fa02bc182310f71bc7d1b1f16611b"; };

  };

}
#endif