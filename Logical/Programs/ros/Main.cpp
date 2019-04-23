
#include <bur/plctypes.h>

#ifdef _DEFAULT_INCLUDES
	#include <AsDefault.h>
#endif

#include "hardware.h"
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "voice_interaction_robot_msgs/FulfilledVoiceCommand.h"
#include "sensor_msgs/JointState.h"

// amount of memory to be allocated for heap storage must be specified for every ANSI C++ program with the bur_heap_size variable
unsigned long bur_heap_size = 0xFFFF; 

// Message callback for the fulfilledCommand subscription
void messageCallback( const voice_interaction_robot_msgs::FulfilledVoiceCommand& fulfilledCommand)
{
	// Post-processing logic to convert the data from any topic subscriptions to useable IEC structures.
	brsmemset((UDINT)&ROSCommand,0,sizeof(ROSCommand));
	if (fulfilledCommand.intent_name != 0)
	{
		// Extracting the intent name
		const char* intent_name = fulfilledCommand.intent_name;	
		brsstrcpy((UDINT)ROSCommand.IntentName,(UDINT)intent_name);
		// Extracting the number of slots
		ROSCommand.NumSlots = (UDINT)fulfilledCommand.slots_length;
		// Now extracting data for each individual slot
		for (UINT i = 0; i < ROS_MAX_SLOT_INDEX; i++)
		{
			if (i < fulfilledCommand.slots_length)
			{
				const char* slot_key = fulfilledCommand.slots[i].key;
				brsstrcpy((UDINT)ROSCommand.Slot[i].Key,(UDINT)slot_key);
				const char* slot_value = fulfilledCommand.slots[i].value;
				brsstrcpy((UDINT)ROSCommand.Slot[i].Value,(UDINT)slot_value);
			}
		}
	}
	return;
}

// Message callback for the joint1State subscription
void jointCallback( const sensor_msgs::JointState& jointData)
{
	joint1ActualPosition = (REAL) jointData.position[0];
	joint2ActualPosition = (REAL) jointData.position[1];
	return;
}

// Create the node handle
ros::NodeHandle_<Hardware> nh;

////************** Handling the string message *********************//
//// Create the message object (just contains a single string)
//std_msgs::String wakeWord;
//// Create the publisher
//ros::Publisher control("/wake_word", &wakeWord);
//
////************** Handling the second string message *********************//
//// Create the message object (just contains a single string)
//std_msgs::String textToSpeak;
//// Create the publisher
//ros::Publisher speaker("/voice_output_node/speak", &textToSpeak);
//
////************** Handling the third string message *********************//
//// Create the message object (just contains a single string)
//std_msgs::String caseTracker;
//// Create the publisher
//ros::Publisher tracker("/case_tracker", &caseTracker);

////************** Handling the FulfilledCommand message *********************//
//// Create my subscriber
//ros::Subscriber<voice_interaction_robot_msgs::FulfilledVoiceCommand> status("/voice_interaction_node/fulfilled_command", messageCallback);

//************** Handling the float64 messages *********************//
// Create the message objects
std_msgs::Float64 joint1Position;
std_msgs::Float64 joint2Position;
// Create the publishers
ros::Publisher joint1Publisher("/rrbot/joint1_position_controller/command", &joint1Position);
ros::Publisher joint2Publisher("/rrbot/joint2_position_controller/command", &joint2Position);

//************** Handling the joint state messages *********************//
// Create my subscribers
ros::Subscriber<sensor_msgs::JointState> jointStates("/rrbot/joint_states", jointCallback);

void _INIT ProgramInit(void)
{	
	nh.initNode();
	nh.setSpinTimeout(2);
//	nh.advertise(control);
//	nh.advertise(speaker);
//	nh.subscribe(status);
//	nh.advertise(tracker);
	nh.subscribe(jointStates);
	nh.advertise(joint1Publisher);
	nh.advertise(joint2Publisher);
}

void _CYCLIC ProgramCyclic(void)
{		
//	if (wakeCommand == true)
//	{
//		wakeWord.data = "wake";
//		control.publish( &wakeWord);
//	}
//	
//	if (speakNow == true)
//	{
//		speakNow = false;
//		char* data = textCommand;
//		textToSpeak.data = data;
//		speaker.publish( &textToSpeak);
//	}
	
	joint1Position.data = (float)joint1SetPosition;	
	joint1Publisher.publish( &joint1Position);
	
//	joint2Position.data = (float)joint2SetPosition;
//	joint2Publisher.publish( &joint2Position);
	
//	if (blueCaseComplete == true) 
//	{
//		blueCaseComplete = false;
//		char* data2 = "Blue case processing complete";
//		caseTracker.data = data2;
//		tracker.publish( &caseTracker);
//	}
//	if (greenCaseComplete == true) 
//	{
//		greenCaseComplete = false;
//		char* data2 = "Green case processing complete";
//		caseTracker.data = data2;
//		tracker.publish( &caseTracker);
//	}
//	if (redCaseComplete == true) 
//	{
//		redCaseComplete = false;
//		char* data2 = "Red case processing complete";
//		caseTracker.data = data2;
//		tracker.publish( &caseTracker);
//	}

//	if (spinContinuous)
	nh.spinOnce();	
	
	recvCounter = nh._recvCounter;
	mode = nh._mode;
	size = nh._msgSize;
	bytes = nh._bytes;
}

void _EXIT ProgramExit(void)
{
	// Insert code here 
}
