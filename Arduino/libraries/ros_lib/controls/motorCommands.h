#ifndef _ROS_controls_motorCommands_h
#define _ROS_controls_motorCommands_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace controls
{

  class motorCommands : public ros::Msg
  {
    public:
      int32_t cmd_surge_starboard;
      int32_t cmd_surge_port;
      int32_t cmd_sway_bow;
      int32_t cmd_sway_stern;
      int32_t cmd_heave_bow;
      int32_t cmd_heave_stern;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_surge_starboard;
      u_cmd_surge_starboard.real = this->cmd_surge_starboard;
      *(outbuffer + offset + 0) = (u_cmd_surge_starboard.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_surge_starboard.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_surge_starboard.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_surge_starboard.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_surge_starboard);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_surge_port;
      u_cmd_surge_port.real = this->cmd_surge_port;
      *(outbuffer + offset + 0) = (u_cmd_surge_port.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_surge_port.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_surge_port.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_surge_port.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_surge_port);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_sway_bow;
      u_cmd_sway_bow.real = this->cmd_sway_bow;
      *(outbuffer + offset + 0) = (u_cmd_sway_bow.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_sway_bow.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_sway_bow.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_sway_bow.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_sway_bow);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_sway_stern;
      u_cmd_sway_stern.real = this->cmd_sway_stern;
      *(outbuffer + offset + 0) = (u_cmd_sway_stern.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_sway_stern.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_sway_stern.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_sway_stern.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_sway_stern);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_heave_bow;
      u_cmd_heave_bow.real = this->cmd_heave_bow;
      *(outbuffer + offset + 0) = (u_cmd_heave_bow.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_heave_bow.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_heave_bow.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_heave_bow.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_heave_bow);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_heave_stern;
      u_cmd_heave_stern.real = this->cmd_heave_stern;
      *(outbuffer + offset + 0) = (u_cmd_heave_stern.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_heave_stern.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_heave_stern.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_heave_stern.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_heave_stern);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_surge_starboard;
      u_cmd_surge_starboard.base = 0;
      u_cmd_surge_starboard.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_surge_starboard.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_surge_starboard.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_surge_starboard.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_surge_starboard = u_cmd_surge_starboard.real;
      offset += sizeof(this->cmd_surge_starboard);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_surge_port;
      u_cmd_surge_port.base = 0;
      u_cmd_surge_port.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_surge_port.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_surge_port.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_surge_port.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_surge_port = u_cmd_surge_port.real;
      offset += sizeof(this->cmd_surge_port);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_sway_bow;
      u_cmd_sway_bow.base = 0;
      u_cmd_sway_bow.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_sway_bow.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_sway_bow.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_sway_bow.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_sway_bow = u_cmd_sway_bow.real;
      offset += sizeof(this->cmd_sway_bow);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_sway_stern;
      u_cmd_sway_stern.base = 0;
      u_cmd_sway_stern.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_sway_stern.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_sway_stern.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_sway_stern.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_sway_stern = u_cmd_sway_stern.real;
      offset += sizeof(this->cmd_sway_stern);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_heave_bow;
      u_cmd_heave_bow.base = 0;
      u_cmd_heave_bow.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_heave_bow.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_heave_bow.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_heave_bow.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_heave_bow = u_cmd_heave_bow.real;
      offset += sizeof(this->cmd_heave_bow);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_heave_stern;
      u_cmd_heave_stern.base = 0;
      u_cmd_heave_stern.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_heave_stern.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_heave_stern.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_heave_stern.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_heave_stern = u_cmd_heave_stern.real;
      offset += sizeof(this->cmd_heave_stern);
     return offset;
    }

    const char * getType(){ return "controls/motorCommands"; };
    const char * getMD5(){ return "ea2d5f40a47b1880926c752b1d9424bf"; };

  };

}
#endif