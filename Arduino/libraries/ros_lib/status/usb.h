#ifndef _ROS_status_usb_h
#define _ROS_status_usb_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace status
{

  class usb : public ros::Msg
  {
    public:
      uint8_t number;
      uint8_t ports_length;
      char* st_ports;
      char* * ports;
      uint8_t name_length;
      char* st_name;
      char* * name;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->number >> (8 * 0)) & 0xFF;
      offset += sizeof(this->number);
      *(outbuffer + offset++) = ports_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < ports_length; i++){
      uint32_t length_portsi = strlen( (const char*) this->ports[i]);
      memcpy(outbuffer + offset, &length_portsi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->ports[i], length_portsi);
      offset += length_portsi;
      }
      *(outbuffer + offset++) = name_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen( (const char*) this->name[i]);
      memcpy(outbuffer + offset, &length_namei, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->number =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->number);
      uint8_t ports_lengthT = *(inbuffer + offset++);
      if(ports_lengthT > ports_length)
        this->ports = (char**)realloc(this->ports, ports_lengthT * sizeof(char*));
      offset += 3;
      ports_length = ports_lengthT;
      for( uint8_t i = 0; i < ports_length; i++){
      uint32_t length_st_ports;
      memcpy(&length_st_ports, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_ports; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_ports-1]=0;
      this->st_ports = (char *)(inbuffer + offset-1);
      offset += length_st_ports;
        memcpy( &(this->ports[i]), &(this->st_ports), sizeof(char*));
      }
      uint8_t name_lengthT = *(inbuffer + offset++);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      offset += 3;
      name_length = name_lengthT;
      for( uint8_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      memcpy(&length_st_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "status/usb"; };
    const char * getMD5(){ return "366c3618c86649340dd416c51dd71b3c"; };

  };

}
#endif