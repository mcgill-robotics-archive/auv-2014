#ifndef _ROS_arduino_msgs_motor_h
#define _ROS_arduino_msgs_motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

  class motor : public ros::Msg
  {
    public:
      int16_t x_p;
      int16_t x_n;
      int16_t y_p;
      int16_t y_n;
      int16_t z_p;
      int16_t z_n;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_x_p;
      u_x_p.real = this->x_p;
      *(outbuffer + offset + 0) = (u_x_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_p.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->x_p);
      union {
        int16_t real;
        uint16_t base;
      } u_x_n;
      u_x_n.real = this->x_n;
      *(outbuffer + offset + 0) = (u_x_n.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_n.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->x_n);
      union {
        int16_t real;
        uint16_t base;
      } u_y_p;
      u_y_p.real = this->y_p;
      *(outbuffer + offset + 0) = (u_y_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_p.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->y_p);
      union {
        int16_t real;
        uint16_t base;
      } u_y_n;
      u_y_n.real = this->y_n;
      *(outbuffer + offset + 0) = (u_y_n.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_n.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->y_n);
      union {
        int16_t real;
        uint16_t base;
      } u_z_p;
      u_z_p.real = this->z_p;
      *(outbuffer + offset + 0) = (u_z_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_p.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->z_p);
      union {
        int16_t real;
        uint16_t base;
      } u_z_n;
      u_z_n.real = this->z_n;
      *(outbuffer + offset + 0) = (u_z_n.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z_n.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->z_n);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_x_p;
      u_x_p.base = 0;
      u_x_p.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_p.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->x_p = u_x_p.real;
      offset += sizeof(this->x_p);
      union {
        int16_t real;
        uint16_t base;
      } u_x_n;
      u_x_n.base = 0;
      u_x_n.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_n.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->x_n = u_x_n.real;
      offset += sizeof(this->x_n);
      union {
        int16_t real;
        uint16_t base;
      } u_y_p;
      u_y_p.base = 0;
      u_y_p.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_p.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->y_p = u_y_p.real;
      offset += sizeof(this->y_p);
      union {
        int16_t real;
        uint16_t base;
      } u_y_n;
      u_y_n.base = 0;
      u_y_n.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_n.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->y_n = u_y_n.real;
      offset += sizeof(this->y_n);
      union {
        int16_t real;
        uint16_t base;
      } u_z_p;
      u_z_p.base = 0;
      u_z_p.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z_p.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->z_p = u_z_p.real;
      offset += sizeof(this->z_p);
      union {
        int16_t real;
        uint16_t base;
      } u_z_n;
      u_z_n.base = 0;
      u_z_n.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z_n.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->z_n = u_z_n.real;
      offset += sizeof(this->z_n);
     return offset;
    }

    const char * getType(){ return "arduino_msgs/motor"; };
    const char * getMD5(){ return "a9af555f873ba8cd2535c14c35157934"; };

  };

}
#endif