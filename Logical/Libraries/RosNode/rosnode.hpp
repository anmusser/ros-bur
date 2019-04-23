#include "hardware.h"

// This is where we include references to all custom message packages
#include "ros/node_handle.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sound_play/SoundRequest.h"
#include "voice_interaction_robot_msgs/FulfilledVoiceCommand.h"

class RosNode
{
	public:
		// Constructor and destructor
		RosNode();
		virtual ~RosNode();
		// Cyclic function for processing all logic
		voice_interaction_robot_msgs::FulfilledVoiceCommand Cyclic(std_msgs::String wakeWord, BOOL wakeCommand);
	
	private:
		// Create the node handle
		ros::NodeHandle_<Hardware> _nh;
	
		// Defining several local PVs
		BOOL _init;
		UDINT _oldCallbackCounter;
};