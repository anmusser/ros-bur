#ifndef _ROS_voice_interaction_robot_msgs_FulfilledVoiceCommand_h
#define _ROS_voice_interaction_robot_msgs_FulfilledVoiceCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"
#include "../diagnostic_msgs/KeyValue.h"

namespace voice_interaction_robot_msgs
{

  class FulfilledVoiceCommand : public ros::Msg
  {
    public:
      typedef const char* _intent_name_type;
      _intent_name_type intent_name;
      uint32_t slots_length;
      typedef diagnostic_msgs::KeyValue _slots_type;
      _slots_type st_slots;
      _slots_type * slots;

    FulfilledVoiceCommand():
      intent_name(""),
      slots_length(0), slots(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_intent_name = strlen(this->intent_name);
      varToArr(outbuffer + offset, length_intent_name);
      offset += 4;
      memcpy(outbuffer + offset, this->intent_name, length_intent_name);
      offset += length_intent_name;
      *(outbuffer + offset + 0) = (this->slots_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->slots_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->slots_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->slots_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->slots_length);
      for( uint32_t i = 0; i < slots_length; i++){
      offset += this->slots[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_intent_name;
      arrToVar(length_intent_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_intent_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_intent_name-1]=0;
      this->intent_name = (char *)(inbuffer + offset-1);
      offset += length_intent_name;
      uint32_t slots_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      slots_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      slots_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      slots_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->slots_length);
      if(slots_lengthT > slots_length)
        this->slots = (diagnostic_msgs::KeyValue*)realloc(this->slots, slots_lengthT * sizeof(diagnostic_msgs::KeyValue));
      slots_length = slots_lengthT;
      for( uint32_t i = 0; i < slots_length; i++){
      offset += this->st_slots.deserialize(inbuffer + offset);
        memcpy( &(this->slots[i]), &(this->st_slots), sizeof(diagnostic_msgs::KeyValue));
      }
     return offset;
    }

    const char * getType(){ return "voice_interaction_robot_msgs/FulfilledVoiceCommand"; };
    const char * getMD5(){ return "ab448516856613a366c554f14df4e422"; };

  };

}
#endif