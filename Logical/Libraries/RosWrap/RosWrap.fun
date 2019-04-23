
{REDUND_ERROR} FUNCTION_BLOCK RosWrapper (*TODO: Add your comment here*) (*$GROUP=User,$CAT=User,$GROUPICON=User.png,$CATICON=User.png*)
	VAR_INPUT
		Enable : {REDUND_UNREPLICABLE} BOOL;
		WakeCommand : BOOL;
	END_VAR
	VAR_OUTPUT
		Active : BOOL;
		Error : BOOL;
		ErrorID : UINT;
		FulfilledCommand : RosWrapper_FulfilledCommand_Type;
	END_VAR
	VAR
		Internal : RosWrapper_Internal_Type;
	END_VAR
END_FUNCTION_BLOCK
