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
      int32_t cmd_x1;
      int32_t cmd_x2;
      int32_t cmd_y1;
      int32_t cmd_y2;
      int32_t cmd_z1;
      int32_t cmd_z2;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_x1;
      u_cmd_x1.real = this->cmd_x1;
      *(outbuffer + offset + 0) = (u_cmd_x1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_x1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_x1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_x1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_x1);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_x2;
      u_cmd_x2.real = this->cmd_x2;
      *(outbuffer + offset + 0) = (u_cmd_x2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_x2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_x2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_x2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_x2);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_y1;
      u_cmd_y1.real = this->cmd_y1;
      *(outbuffer + offset + 0) = (u_cmd_y1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_y1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_y1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_y1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_y1);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_y2;
      u_cmd_y2.real = this->cmd_y2;
      *(outbuffer + offset + 0) = (u_cmd_y2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_y2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_y2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_y2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_y2);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_z1;
      u_cmd_z1.real = this->cmd_z1;
      *(outbuffer + offset + 0) = (u_cmd_z1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_z1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_z1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_z1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_z1);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_z2;
      u_cmd_z2.real = this->cmd_z2;
      *(outbuffer + offset + 0) = (u_cmd_z2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd_z2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd_z2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd_z2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd_z2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_x1;
      u_cmd_x1.base = 0;
      u_cmd_x1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_x1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_x1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_x1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_x1 = u_cmd_x1.real;
      offset += sizeof(this->cmd_x1);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_x2;
      u_cmd_x2.base = 0;
      u_cmd_x2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_x2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_x2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_x2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_x2 = u_cmd_x2.real;
      offset += sizeof(this->cmd_x2);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_y1;
      u_cmd_y1.base = 0;
      u_cmd_y1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_y1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_y1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_y1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_y1 = u_cmd_y1.real;
      offset += sizeof(this->cmd_y1);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_y2;
      u_cmd_y2.base = 0;
      u_cmd_y2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_y2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_y2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_y2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_y2 = u_cmd_y2.real;
      offset += sizeof(this->cmd_y2);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_z1;
      u_cmd_z1.base = 0;
      u_cmd_z1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_z1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_z1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_z1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_z1 = u_cmd_z1.real;
      offset += sizeof(this->cmd_z1);
      union {
        int32_t real;
        uint32_t base;
      } u_cmd_z2;
      u_cmd_z2.base = 0;
      u_cmd_z2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd_z2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd_z2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd_z2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd_z2 = u_cmd_z2.real;
      offset += sizeof(this->cmd_z2);
     return offset;
    }

    const char * getType(){ return "controls/motorCommands"; };
    const char * getMD5(){ return "5edbe2b4897bc11e5f0f40b64b7cffdd"; };

  };

}
#endif