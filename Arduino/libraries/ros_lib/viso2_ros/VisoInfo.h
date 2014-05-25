#ifndef _ROS_viso2_ros_VisoInfo_h
#define _ROS_viso2_ros_VisoInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace viso2_ros
{

  class VisoInfo : public ros::Msg
  {
    public:
      std_msgs::Header header;
      bool got_lost;
      bool change_reference_frame;
      bool motion_estimate_valid;
      int32_t num_matches;
      int32_t num_inliers;
      float runtime;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_got_lost;
      u_got_lost.real = this->got_lost;
      *(outbuffer + offset + 0) = (u_got_lost.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->got_lost);
      union {
        bool real;
        uint8_t base;
      } u_change_reference_frame;
      u_change_reference_frame.real = this->change_reference_frame;
      *(outbuffer + offset + 0) = (u_change_reference_frame.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->change_reference_frame);
      union {
        bool real;
        uint8_t base;
      } u_motion_estimate_valid;
      u_motion_estimate_valid.real = this->motion_estimate_valid;
      *(outbuffer + offset + 0) = (u_motion_estimate_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motion_estimate_valid);
      union {
        int32_t real;
        uint32_t base;
      } u_num_matches;
      u_num_matches.real = this->num_matches;
      *(outbuffer + offset + 0) = (u_num_matches.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_matches.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_matches.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_matches.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_matches);
      union {
        int32_t real;
        uint32_t base;
      } u_num_inliers;
      u_num_inliers.real = this->num_inliers;
      *(outbuffer + offset + 0) = (u_num_inliers.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_num_inliers.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_num_inliers.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_num_inliers.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->num_inliers);
      int32_t * val_runtime = (int32_t *) &(this->runtime);
      int32_t exp_runtime = (((*val_runtime)>>23)&255);
      if(exp_runtime != 0)
        exp_runtime += 1023-127;
      int32_t sig_runtime = *val_runtime;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = (sig_runtime<<5) & 0xff;
      *(outbuffer + offset++) = (sig_runtime>>3) & 0xff;
      *(outbuffer + offset++) = (sig_runtime>>11) & 0xff;
      *(outbuffer + offset++) = ((exp_runtime<<4) & 0xF0) | ((sig_runtime>>19)&0x0F);
      *(outbuffer + offset++) = (exp_runtime>>4) & 0x7F;
      if(this->runtime < 0) *(outbuffer + offset -1) |= 0x80;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_got_lost;
      u_got_lost.base = 0;
      u_got_lost.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->got_lost = u_got_lost.real;
      offset += sizeof(this->got_lost);
      union {
        bool real;
        uint8_t base;
      } u_change_reference_frame;
      u_change_reference_frame.base = 0;
      u_change_reference_frame.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->change_reference_frame = u_change_reference_frame.real;
      offset += sizeof(this->change_reference_frame);
      union {
        bool real;
        uint8_t base;
      } u_motion_estimate_valid;
      u_motion_estimate_valid.base = 0;
      u_motion_estimate_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->motion_estimate_valid = u_motion_estimate_valid.real;
      offset += sizeof(this->motion_estimate_valid);
      union {
        int32_t real;
        uint32_t base;
      } u_num_matches;
      u_num_matches.base = 0;
      u_num_matches.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_matches.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_matches.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_matches.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_matches = u_num_matches.real;
      offset += sizeof(this->num_matches);
      union {
        int32_t real;
        uint32_t base;
      } u_num_inliers;
      u_num_inliers.base = 0;
      u_num_inliers.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_num_inliers.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_num_inliers.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_num_inliers.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->num_inliers = u_num_inliers.real;
      offset += sizeof(this->num_inliers);
      uint32_t * val_runtime = (uint32_t*) &(this->runtime);
      offset += 3;
      *val_runtime = ((uint32_t)(*(inbuffer + offset++))>>5 & 0x07);
      *val_runtime |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<3;
      *val_runtime |= ((uint32_t)(*(inbuffer + offset++)) & 0xff)<<11;
      *val_runtime |= ((uint32_t)(*(inbuffer + offset)) & 0x0f)<<19;
      uint32_t exp_runtime = ((uint32_t)(*(inbuffer + offset++))&0xf0)>>4;
      exp_runtime |= ((uint32_t)(*(inbuffer + offset)) & 0x7f)<<4;
      if(exp_runtime !=0)
        *val_runtime |= ((exp_runtime)-1023+127)<<23;
      if( ((*(inbuffer+offset++)) & 0x80) > 0) this->runtime = -this->runtime;
     return offset;
    }

    const char * getType(){ return "viso2_ros/VisoInfo"; };
    const char * getMD5(){ return "765500d8b83bf74f7715c6e2e8e89092"; };

  };

}
#endif