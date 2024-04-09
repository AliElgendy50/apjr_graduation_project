#ifndef _ROS_apjr_control_VelocitiesMsg_h
#define _ROS_apjr_control_VelocitiesMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace apjr_control
{

  class VelocitiesMsg : public ros::Msg
  {
    public:
      typedef float _linearVx_type;
      _linearVx_type linearVx;
      typedef float _linearVy_type;
      _linearVy_type linearVy;
      typedef float _angularVz_type;
      _angularVz_type angularVz;

    VelocitiesMsg():
      linearVx(0),
      linearVy(0),
      angularVz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_linearVx;
      u_linearVx.real = this->linearVx;
      *(outbuffer + offset + 0) = (u_linearVx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linearVx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linearVx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linearVx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linearVx);
      union {
        float real;
        uint32_t base;
      } u_linearVy;
      u_linearVy.real = this->linearVy;
      *(outbuffer + offset + 0) = (u_linearVy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linearVy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linearVy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linearVy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linearVy);
      union {
        float real;
        uint32_t base;
      } u_angularVz;
      u_angularVz.real = this->angularVz;
      *(outbuffer + offset + 0) = (u_angularVz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angularVz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angularVz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angularVz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angularVz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_linearVx;
      u_linearVx.base = 0;
      u_linearVx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linearVx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linearVx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linearVx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linearVx = u_linearVx.real;
      offset += sizeof(this->linearVx);
      union {
        float real;
        uint32_t base;
      } u_linearVy;
      u_linearVy.base = 0;
      u_linearVy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linearVy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linearVy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linearVy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linearVy = u_linearVy.real;
      offset += sizeof(this->linearVy);
      union {
        float real;
        uint32_t base;
      } u_angularVz;
      u_angularVz.base = 0;
      u_angularVz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angularVz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angularVz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angularVz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angularVz = u_angularVz.real;
      offset += sizeof(this->angularVz);
     return offset;
    }

    virtual const char * getType() override { return "apjr_control/VelocitiesMsg"; };
    virtual const char * getMD5() override { return "1a7825a79335720cf5038a024bbd0c41"; };

  };

}
#endif
