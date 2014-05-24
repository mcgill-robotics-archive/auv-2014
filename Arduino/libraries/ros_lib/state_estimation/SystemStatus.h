#ifndef _ROS_state_estimation_SystemStatus_h
#define _ROS_state_estimation_SystemStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace state_estimation
{

  class SystemStatus : public ros::Msg
  {
    public:
      char * topic_name;
      char * status;
      int32_t alert_level;
      enum { ALERT_GREEN = 0 };
      enum { ALERT_YELLOW = 1 };
      enum { ALERT_RED = 2 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_topic_name = strlen( (const char*) this->topic_name);
      memcpy(outbuffer + offset, &length_topic_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->topic_name, length_topic_name);
      offset += length_topic_name;
      uint32_t length_status = strlen( (const char*) this->status);
      memcpy(outbuffer + offset, &length_status, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->status, length_status);
      offset += length_status;
      union {
        int32_t real;
        uint32_t base;
      } u_alert_level;
      u_alert_level.real = this->alert_level;
      *(outbuffer + offset + 0) = (u_alert_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_alert_level.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_alert_level.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_alert_level.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->alert_level);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_topic_name;
      memcpy(&length_topic_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_topic_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_topic_name-1]=0;
      this->topic_name = (char *)(inbuffer + offset-1);
      offset += length_topic_name;
      uint32_t length_status;
      memcpy(&length_status, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status-1]=0;
      this->status = (char *)(inbuffer + offset-1);
      offset += length_status;
      union {
        int32_t real;
        uint32_t base;
      } u_alert_level;
      u_alert_level.base = 0;
      u_alert_level.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_alert_level.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_alert_level.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_alert_level.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->alert_level = u_alert_level.real;
      offset += sizeof(this->alert_level);
     return offset;
    }

    const char * getType(){ return "state_estimation/SystemStatus"; };
    const char * getMD5(){ return "2b7987778e72eaea188a7bba683f7ba3"; };

  };

}
#endif