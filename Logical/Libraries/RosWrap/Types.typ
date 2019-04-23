
TYPE
	RosWrapper_Internal_Type : 	STRUCT 
		Ident : UDINT;
		OldWakeCommand : BOOL;
	END_STRUCT;
	RosWrapper_Velocity_Type : 	STRUCT 
		linear : Vector3_Type;
		angular : Vector3_Type;
	END_STRUCT;
	Vector3_Type : 	STRUCT 
		x : REAL;
		y : REAL;
		z : REAL;
	END_STRUCT;
	RosWrapper_Slot_Type : 	STRUCT 
		Key : STRING[80];
		Value : STRING[80];
	END_STRUCT;
	RosWrapper_FulfilledCommand_Type : 	STRUCT 
		IntentName : STRING[80];
		NumSlots : UDINT;
		Slot : ARRAY[0..ROS_MAX_SLOT_INDEX]OF RosWrapper_Slot_Type;
	END_STRUCT;
END_TYPE
