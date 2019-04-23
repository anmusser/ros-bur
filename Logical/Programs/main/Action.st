
ACTION ProcessROSCommands:
	
	// Command to turn the work light on or off
	IF (ROSCommand.IntentName = 'workLightCommand') THEN
		IF (ROSCommand.Slot[0].Value = 'off') THEN
			WorklightOn := FALSE;
		ELSIF (ROSCommand.Slot[0].Value = 'on') THEN
			WorklightOn := TRUE;
		END_IF
	END_IF
	
	// Command to jog the axis
	IF (ROSCommand.IntentName = 'jogMove') THEN
		IF (ROSCommand.Slot[0].Value = 'forward') THEN
			ConveyorParameters.Direction := mpAXIS_DIR_POSITIVE;
		ELSIF (ROSCommand.Slot[0].Value = 'backward') THEN
			ConveyorParameters.Direction := mpAXIS_DIR_NEGATIVE;
		END_IF
		ConveyorParameters.Velocity := brsatof(ADR(ROSCommand.Slot[1].Value));
		Conveyor.MoveVelocity := TRUE;
	END_IF	
	
	// Command to move a fixed distance
	IF (ROSCommand.IntentName = 'distanceMove') THEN
		IF (ROSCommand.Slot[0].Value = 'forward') THEN
			ConveyorParameters.Direction := mpAXIS_DIR_POSITIVE;
			ConveyorParameters.Distance := 1 * brsatof(ADR(ROSCommand.Slot[1].Value));
		ELSIF (ROSCommand.Slot[0].Value = 'backward') THEN
			ConveyorParameters.Distance := (-1) * brsatof(ADR(ROSCommand.Slot[1].Value));
			ConveyorParameters.Direction := mpAXIS_DIR_NEGATIVE;
		END_IF
		Conveyor.MoveAdditive := TRUE;
	END_IF	
	
	// Stop command--halts all motion
	IF (ROSCommand.IntentName = 'stop') THEN
		Conveyor.MoveVelocity := FALSE;
		Conveyor.MoveAbsolute := FALSE;
		Conveyor.MoveAdditive := FALSE;
		StartAutoMode := FALSE;
	END_IF
	
	// Command to change parameter values
	IF (ROSCommand.IntentName = 'setParameter') THEN
		IF (ROSCommand.Slot[0].Value = 'speed') THEN
			ConveyorParameters.Velocity := brsatof(ADR(ROSCommand.Slot[1].Value));
		ELSIF (ROSCommand.Slot[0].Value = 'acceleration') THEN
			ConveyorParameters.Acceleration := brsatof(ADR(ROSCommand.Slot[1].Value));
		ELSIF (ROSCommand.Slot[0].Value = 'deceleration') THEN
			ConveyorParameters.Deceleration := brsatof(ADR(ROSCommand.Slot[1].Value));
		END_IF
	END_IF
	
	// Case commands: place a case
	IF (ROSCommand.IntentName = 'placeCase') THEN
		IF (ROSCommand.Slot[0].Value = 'Blue') THEN
			ShowBox1 := TRUE;
		ELSIF (ROSCommand.Slot[0].Value = 'Green') THEN
			ShowBox2 := TRUE;
		ELSIF (ROSCommand.Slot[0].Value = 'red') THEN
			ShowBox3 := TRUE;
		END_IF
	END_IF
	// Case commands: clear all cases
	IF (ROSCommand.IntentName = 'clearAllCases') THEN
		ShowBox1 := FALSE;
		ShowBox2 := FALSE;
		ShowBox3 := FALSE;
	END_IF
	// Case commands: determine case position
	IF (ROSCommand.IntentName = 'caseLocator') THEN
		IF (ROSCommand.Slot[0].Value = 'Blue') THEN
			IF (ShowBox1) THEN
				textCommand := 'The blue case is ';
				brsftoa(BoxPosition1,ADR(TempString));
				brsstrcat(ADR(textCommand),ADR(TempString));
				brsstrcat(ADR(textCommand),ADR(' meters away from the infeed.'));
				speakNow := TRUE;
			ELSE
				textCommand := 'The blue case is not present.';
				speakNow := TRUE;
			END_IF
		ELSIF (ROSCommand.Slot[0].Value = 'Green') THEN
			IF (ShowBox2) THEN
				textCommand := 'The green case is ';
				brsftoa(BoxPosition2,ADR(TempString));
				brsstrcat(ADR(textCommand),ADR(TempString));
				brsstrcat(ADR(textCommand),ADR(' meters away from the infeed.'));
				speakNow := TRUE;
			ELSE
				textCommand := 'The green case is not present.';
				speakNow := TRUE;
			END_IF
		ELSIF (ROSCommand.Slot[0].Value = 'red') THEN
			IF (ShowBox3) THEN
				textCommand := 'The red case is ';
				brsftoa(BoxPosition3,ADR(TempString));
				brsstrcat(ADR(textCommand),ADR(TempString));
				brsstrcat(ADR(textCommand),ADR(' meters away from the infeed.'));
				speakNow := TRUE;
			ELSE
				textCommand := 'The red case is not present.';
				speakNow := TRUE;
			END_IF
		END_IF
	END_IF
	
	// Notification prompt for photoeye
	IF (ROSCommand.IntentName = 'notificationPrompt') THEN
		IF (ROSCommand.Slot[0].Value = 'Blue') THEN
			NotifyBox1 := TRUE;
		ELSIF (ROSCommand.Slot[0].Value = 'Green') THEN
			NotifyBox2 := TRUE;
		ELSIF (ROSCommand.Slot[0].Value = 'red') THEN
			NotifyBox3 := TRUE;
		END_IF
	END_IF
	
	// Data retrieval command
	IF (ROSCommand.IntentName = 'speedLocator') THEN
		IF (Conveyor.Velocity = 0) THEN
			textCommand := 'The conveyor is not currently moving.';
		ELSE
			textCommand := 'The current conveyor speed is ';
			brsftoa(Conveyor.Velocity,ADR(TempString));
			brsstrcat(ADR(textCommand),ADR(TempString));
			brsstrcat(ADR(textCommand),ADR(' meters per second.'));
		END_IF
		speakNow := TRUE;
	END_IF
	brsmemset(ADR(ROSCommand),0,SIZEOF(ROSCommand));

	// Delayed speech actions
	IF ((NotifyBox1) AND (BoxPosition1 > NOTIFY_POSITION)) THEN
		NotifyBox1 := FALSE;
		textCommand := 'The blue case has passed the photo eye.';
		speakNow := TRUE;
	END_IF	
	IF ((NotifyBox2) AND (BoxPosition2 > NOTIFY_POSITION)) THEN
		NotifyBox2 := FALSE;
		textCommand := 'The green case has passed the photo eye.';
		speakNow := TRUE;
	END_IF
	IF ((NotifyBox3) AND (BoxPosition3 > NOTIFY_POSITION)) THEN
		NotifyBox3 := FALSE;
		textCommand := 'The red case has passed the photo eye.';
		speakNow := TRUE;
	END_IF

END_ACTION
