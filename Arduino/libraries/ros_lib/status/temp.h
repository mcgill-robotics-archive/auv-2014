#ifndef _ROS_status_temp_h
#define _ROS_status_temp_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace status
{

  class temp : public ros::Msg
  {
    public:
      uint8_t core_0;
      uint8_t core_1;
      uint8_t core_2;
      uint8_t core_3;
      uint8_t ssd;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->core_0 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->core_0);
      *(outbuffer + offset + 0) = (this->core_1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->core_1);
      *(outbuffer + offset + 0) = (this->core_2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->core_2);
      *(outbuffer + offset + 0) = (this->core_3 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->core_3);
      *(outbuffer + offset + 0) = (this->ssd >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ssd);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->core_0 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->core_0);
      this->core_1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->core_1);
      this->core_2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->core_2);
      this->core_3 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->core_3);
      this->ssd =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ssd);
     return offset;
    }

    const char * getType(){ return "status/temp"; };
    const char * getMD5(){ return "89383f5a547720a5d5829a0180f332c5"; };

  };

}
#endif