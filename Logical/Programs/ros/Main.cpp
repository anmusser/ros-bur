
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

// Message callback for the joint1State subscription
void jointCallback( const sensor_msgs::JointState& jointData)
{
	joint1ActualPosition = (REAL) jointData.position[0];
	joint2ActualPosition = (REAL) jointData.position[1];
	return;
}

// Create the node handle
ros::NodeHandle_<Hardware> nh;

//************** Handling the float64 messages *********************//
// Create the message objects
std_msgs::Float64 joint1Position;
// Create the publishers
ros::Publisher joint1Publisher("/rrbot/joint1_position_controller/command", &joint1Position);

//************** Handling the joint state messages *********************//
// Create my subscribers
ros::Subscriber<sensor_msgs::JointState> jointStates("/rrbot/joint_states", jointCallback);

void _INIT ProgramInit(void)
{	
	nh.initNode();
	nh.subscribe(jointStates);
	nh.advertise(joint1Publisher);
}

void _CYCLIC ProgramCyclic(void)
{			
	joint1Position.data = (float)joint1SetPosition;	
	joint1Publisher.publish( &joint1Position);

	nh.spinOnce();	
}

void _EXIT ProgramExit(void)
{
	// Insert code here 
}
