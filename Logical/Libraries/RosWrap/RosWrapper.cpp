#include <bur/plctypes.h>
#include "RosWrap.h"
#include "../RosNode/rosnode.hpp"

// amount of memory to be allocated for heap storage must be specified for every ANSI C++ program with the bur_heap_size variable
unsigned long bur_heap_size = 0xFFFF; 

void RosWrapper(struct RosWrapper* Inst)
{	
	static std_msgs::String wakeWord;
	static BOOL wakeCommand;
	
	static voice_interaction_robot_msgs::FulfilledVoiceCommand fulfilledCommand;
	
	if (Inst == 0)
		return;
    
	if (Inst->Enable == true)
	{
		// The first time the FB gets here after being enabled, setup operations are carried out
		if (Inst->Internal.Ident == 0)
		{
			// Constructing the RosNode object based on the input configuration info
			RosNode* NodeInstance = new RosNode();
			Inst->Internal.Ident = reinterpret_cast<UDINT>(NodeInstance);
		}
		else
		{			
			// Dereferencing the axis object
			RosNode* NodeInstance = static_cast<RosNode*>((void*)Inst->Internal.Ident);
			
//			// Mapping parameters from outside world.
//			if ((Inst->WakeCommand) && (!Inst->Internal.OldWakeCommand))
//			{
//				wakeCommand = true;
//				wakeWord.data = "wake";
//			} 
//			else
//			{
//				wakeCommand = false;
//				wakeWord.data = "";
//			}
			
			// Calling the cyclic processing method. 
			fulfilledCommand = NodeInstance->Cyclic(wakeWord,wakeCommand);
//			NodeInstance->Cyclic();
		
			// Post-processing logic to convert the data from any topic subscriptions to useable IEC structures.
			brsmemset((UDINT)&Inst->FulfilledCommand,0,sizeof(Inst->FulfilledCommand));
			if (fulfilledCommand.intent_name != 0)
			{
				// Extracting the intent name
				const char* intent_name = fulfilledCommand.intent_name;	
				brsstrcpy((UDINT)Inst->FulfilledCommand.IntentName,(UDINT)intent_name);
				// Extracting the number of slots
				Inst->FulfilledCommand.NumSlots = (UDINT)fulfilledCommand.slots_length;
				// Now extracting data for each individual slot
				for (UINT i = 0; i < ROS_MAX_SLOT_INDEX; i++)
				{
					if (i < fulfilledCommand.slots_length)
					{
						const char* slot_key = fulfilledCommand.slots[i].key;
						brsstrcpy((UDINT)Inst->FulfilledCommand.Slot[i].Key,(UDINT)slot_key);
						const char* slot_value = fulfilledCommand.slots[i].value;
						brsstrcpy((UDINT)Inst->FulfilledCommand.Slot[i].Value,(UDINT)slot_value);
					}
				}
			}
//			
//			// Storing old values for edge detection
//			Inst->Internal.OldWakeCommand = Inst->WakeCommand;
		}
	}			
}
