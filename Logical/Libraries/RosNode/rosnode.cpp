#include "rosnode.hpp"

// Constructor
RosNode::RosNode()
{
	_nh.initNode();
	_nh.setSpinTimeout(1);
}

// Destructor
RosNode::~RosNode()
{
}

// Message callback for the fulfilledCommand subscription
voice_interaction_robot_msgs::FulfilledVoiceCommand messageCallback( const voice_interaction_robot_msgs::FulfilledVoiceCommand& fulfilledCommand)
{
	return fulfilledCommand;
}

// Cyclic execution logic
voice_interaction_robot_msgs::FulfilledVoiceCommand RosNode::Cyclic(std_msgs::String wakeWord, BOOL wakeCommand)
{	
	// Create my publisher
	static ros::Publisher control("/wake_word", &wakeWord);
	
	// Create my subscriber
	static ros::Subscriber<voice_interaction_robot_msgs::FulfilledVoiceCommand> status("/voice_interaction_node/fulfilled_command", messageCallback);
	
	// Initialization logic for publishers and subscribers
	if (!_init)
	{	
		_nh.advertise(control);
		_nh.subscribe(status);
		_init = true;
	}

	// We only publish the wakeWord if commanded to do so (i.e. not cyclic publishing)
	if (wakeCommand == true)
	control.publish( &wakeWord);
	
	// This call runs all of the underlying node's cyclic logic. 
	_nh.spinOnce();
	
	// Keeping track of new data received on the callback function, and returning this data
	if (status.callbackCounter_ != _oldCallbackCounter)
	{
		_oldCallbackCounter = status.callbackCounter_;
		return status.fulfilledCommand_;	
	} else
	{
	_oldCallbackCounter = status.callbackCounter_;
	voice_interaction_robot_msgs::FulfilledVoiceCommand dummyMessage;
	brsmemset((UDINT)&dummyMessage,0,sizeof(dummyMessage));
	return dummyMessage;
	}
};