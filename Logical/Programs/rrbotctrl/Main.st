
PROGRAM _INIT
	
	RRBotJ1.Axis := ADR(gRRBotJ1);
	RRBotJ1.MpLink := ADR(gMpLinkRRBotJ1);
	RRBotJ1.Parameters := ADR(RRBotJ1Parameters);
	RRBotJ1.Enable := TRUE;
	 
END_PROGRAM

PROGRAM _CYCLIC
	
	IF (RRBotJ1.Active) THEN
		RRBotJ1.Power := TRUE;
	END_IF
	IF (RRBotJ1.PowerOn) AND (NOT RRBotJ1.IsHomed) THEN
		RRBotJ1.Home := TRUE;
	END_IF
	IF (RRBotJ1Parameters.Velocity <> OldParameters.Velocity) THEN
		RRBotJ1.Update := TRUE;
	END_IF
	IF (RRBotJ1.UpdateDone) THEN
		RRBotJ1.Update := FALSE;
	END_IF
	
	RRBotJ1();
	
	joint1SetPosition := LREAL_TO_REAL((RRBotJ1.Position) * PI / 180);
	
	joint1ActualPosition;
	joint2ActualPosition;
	
	OldParameters := RRBotJ1Parameters;	
	
	// Handling the motion of the motors
	IF (RRBotJ1.IsHomed) THEN
		// Logic to figure out where joint 1 is
		joint1Position := REAL_TO_UINT((joint1ActualPosition * 180 / PI) + 180) MOD 360;
		brsitoa(joint1Position,ADR(PositionString));
		joint1Transform := '{"select":"#joint1Group","spin":[';
		brsstrcat(ADR(joint1Transform),ADR(PositionString));
		brsstrcat(ADR(joint1Transform),ADR(',10,0]}'));		
		// Logic to figure out where joint 2 is
		joint2Position := REAL_TO_UINT(joint2ActualPosition * 180 / PI) MOD 360;
		brsitoa(joint2Position,ADR(PositionString));
		joint2Transform := '{"select":"#joint2Group","spin":[';
		brsstrcat(ADR(joint2Transform),ADR(PositionString));
		brsstrcat(ADR(joint2Transform),ADR(',10,202]}'));
	ELSE
		joint1Transform := '{"select":"#joint1Group","spin":[0,10,0]}';
		joint2Transform := '{"select":"#joint2Group","spin":[0,10,202]}';	
	END_IF
	
	// Creating the transform string from all elements
	totalTransform := '[';
	brsstrcat(ADR(totalTransform),ADR(joint1Transform));
	brsstrcat(ADR(totalTransform),ADR(','));
	brsstrcat(ADR(totalTransform),ADR(joint2Transform));
	brsstrcat(ADR(totalTransform),ADR(']'));
	
	joint1PositionDegrees := fmod((joint1ActualPosition * 360 / (2 * 3.14159)) + 360, 360);
	joint2PositionDegrees := fmod((joint2ActualPosition * 360 / (2 * 3.14159)) + 360, 360);
	 
END_PROGRAM

PROGRAM _EXIT
	(* Insert code here *)
	 
END_PROGRAM

