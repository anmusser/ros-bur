
PROGRAM _INIT
	
	Conveyor.MpLink := ADR(gMpLinkConveyor);
	Conveyor.Axis := ADR(gConveyor);
	Conveyor.Enable := TRUE;
	Conveyor.Parameters := ADR(ConveyorParameters);
	
	ConveyorParameters.Acceleration := 1;
	ConveyorParameters.Deceleration := 1;
	ConveyorParameters.Jog.Acceleration := ConveyorParameters.Acceleration;
	ConveyorParameters.Jog.Deceleration := ConveyorParameters.Deceleration;
	ConveyorParameters.Jog.Velocity := 1;
	ConveyorParameters.Velocity := 1;
	
	TON_ShowBox1.PT := T#1.75s;
	TON_ShowBox2.PT := T#1.75s;
	TON_ShowBox3.PT := T#1.75s;
	 
END_PROGRAM

PROGRAM _CYCLIC
	
	// Scaling from axis units to HMI: 100 pixels = 1 meter. 
	// On our SVG, the outer belt has a diameter of 80 pixels. This means that one rev of the belt => 251.3 pixels => 2.513 meters	
	
	ProcessROSCommands;
	
	// Initializing our axis
	IF (Conveyor.Active) THEN
		Conveyor.Power := TRUE;
		IF (Conveyor.PowerOn) THEN
			Conveyor.Home := TRUE;
		END_IF
		IF (Conveyor.IsHomed) THEN
			Conveyor.Home := FALSE;
		END_IF
	END_IF
	IF (Conveyor.Info.MoveDone) THEN
		Conveyor.MoveAbsolute := FALSE;
		Conveyor.MoveAdditive := FALSE;
	END_IF
	
	// Detect parameter changes and update on the fly
	IF (((ConveyorParameters.Velocity <> OldParameters.Velocity) OR (ConveyorParameters.Acceleration <> OldParameters.Acceleration) OR
		(ConveyorParameters.Deceleration <> OldParameters.Deceleration)) AND (Conveyor.MoveActive)) THEN
		Conveyor.Update := TRUE;
	END_IF
	IF (Conveyor.UpdateDone) THEN
		Conveyor.Update := FALSE;
	END_IF
	OldParameters := ConveyorParameters;
	
	CASE AutoStep OF
		STEP_IDLE:
			IF (StartAutoMode) THEN
				ShowBox1 := TRUE;
				NextBoxIndex := 1;
				Conveyor.MoveVelocity := TRUE;
				AutoStep := STEP_AUTO;
			END_IF
			
		STEP_AUTO:
			IF (NOT StartAutoMode) THEN
				Conveyor.MoveVelocity := FALSE;
				AutoStep := STEP_IDLE;
			ELSIF (NextBoxPosition > 3) THEN
				IF (NextBoxIndex = 1) THEN
					ShowBox2 := TRUE;
				ELSIF (NextBoxIndex = 2) THEN
					ShowBox3 := TRUE;
				ELSIF (NextBoxIndex = 3) THEN
					ShowBox1 := TRUE;
				END_IF
				NextBoxIndex := NextBoxIndex + 1;
			END_IF
			
	END_CASE
	
	IF (NextBoxIndex = 1) THEN
		NextBoxPosition := BoxPosition1;
	ELSIF (NextBoxIndex = 2) THEN
		NextBoxPosition := BoxPosition2;
	ELSIF (NextBoxIndex = 3) THEN
		NextBoxPosition := BoxPosition3;
	END_IF
	
	// Handling the motion of the motors
	IF (Conveyor.IsHomed) THEN
		// Logic to figure out where the motors are
		//	MotorPosition := (360 * fmod(LREAL_TO_REAL(Conveyor.Position),MOTOR_PERIOD) / MOTOR_PERIOD);
		MotorPosition := (360 * LREAL_TO_REAL(Conveyor.Position) / MOTOR_PERIOD);
		brsftoa(MotorPosition,ADR(PositionString));
		Motor1Transform := '{"select":"#motorAxis1","duration":2,"spin":[';
		brsstrcat(ADR(Motor1Transform),ADR(PositionString));
		brsstrcat(ADR(Motor1Transform),ADR(',0,0]}'));
		Motor2Transform := '{"select":"#motorAxis2","duration":2,"spin":[';
		brsstrcat(ADR(Motor2Transform),ADR(PositionString));
		brsstrcat(ADR(Motor2Transform),ADR(',0,0]}'));
		Motor3Transform := '{"select":"#motorAxis3","duration":2,"spin":[';
		brsstrcat(ADR(Motor3Transform),ADR(PositionString));
		brsstrcat(ADR(Motor3Transform),ADR(',0,0]}'));
		Motor4Transform := '{"select":"#motorAxis4","duration":2,"spin":[';
		brsstrcat(ADR(Motor4Transform),ADR(PositionString));
		brsstrcat(ADR(Motor4Transform),ADR(',0,0]}'));
	ELSE	
		Motor1Transform := '{"select":"#motorAxis1","duration":2,"spin":[0,0,0]}';
		Motor2Transform := '{"select":"#motorAxis2","duration":2,"spin":[0,0,0]}';
		Motor3Transform := '{"select":"#motorAxis3","duration":2,"spin":[0,0,0]}';
		Motor4Transform := '{"select":"#motorAxis4","duration":2,"spin":[0,0,0]}';
	END_IF
	
	// Handling the motion of the boxes
	IF (EDGEPOS(ShowBox1)) THEN
		TON_ShowBox1.IN := TRUE;
	END_IF
	IF (TON_ShowBox1.Q) THEN
		TON_ShowBox1.IN := FALSE;
		BoxOffset1 := Conveyor.Position;
	END_IF
	IF (TON_ShowBox1.IN) THEN
		Box1Transform := '{"select":"#box_blue_1","display":true,"duration":1500,"scale":[1.2,1.2],"translate":[0,0]}';
	ELSIF (ShowBox1) THEN
		BoxPosition1 := LREAL_TO_REAL(Conveyor.Position - BoxOffset1);
		Box1Transform := '{"select":"#box_blue_1","display":true,"duration":2,"scale":[1,1],"translate":[';
		brsftoa(LREAL_TO_REAL(BoxPosition1*100),ADR(PositionString));
		brsstrcat(ADR(Box1Transform),ADR(PositionString));
		brsstrcat(ADR(Box1Transform),ADR(',0]}'));
	ELSE
		BoxPosition1 := 0;
		Box1Transform := Box2Transform := '{"select":"#box_blue_1","display":false,"duration":2,"scale":[0.1,0.1],"translate":[40,0]}';	
	END_IF
	IF (BoxPosition1 > EXIT_POSITION) THEN
		ShowBox1 := FALSE;
		blueCaseComplete := TRUE;
	END_IF

	IF (EDGEPOS(ShowBox2)) THEN
		TON_ShowBox2.IN := TRUE;
	END_IF
	IF (TON_ShowBox2.Q) THEN
		TON_ShowBox2.IN := FALSE;
		BoxOffset2 := Conveyor.Position;
	END_IF
	IF (TON_ShowBox2.IN) THEN
		Box2Transform := '{"select":"#box_green_2","display":true,"duration":1500,"scale":[1.2,1.2],"translate":[0,0]}';
	ELSIF (ShowBox2) THEN
		BoxPosition2 := LREAL_TO_REAL(Conveyor.Position - BoxOffset2);
		Box2Transform := '{"select":"#box_green_2","display":true,"duration":2,"scale":[1,1],"translate":[';
		brsftoa(LREAL_TO_REAL(BoxPosition2*100),ADR(PositionString));
		brsstrcat(ADR(Box2Transform),ADR(PositionString));
		brsstrcat(ADR(Box2Transform),ADR(',0]}'));
	ELSE
		BoxPosition2 := 0;
		Box2Transform := '{"select":"#box_green_2","display":false,"duration":2,"scale":[0.1,0.1],"translate":[30,0]}';		
	END_IF
	
	IF (BoxPosition2 > EXIT_POSITION) THEN
		ShowBox2 := FALSE;
		greenCaseComplete := TRUE;
	END_IF
	
	IF (EDGEPOS(ShowBox3)) THEN
		TON_ShowBox3.IN := TRUE;
	END_IF
	IF (TON_ShowBox3.Q) THEN
		TON_ShowBox3.IN := FALSE;
		BoxOffset3 := Conveyor.Position;
	END_IF
	IF (TON_ShowBox3.IN) THEN
		Box3Transform := '{"select":"#box_red_3","display":true,"duration":1500,"scale":[1.2,1.2],"translate":[0,0]}';
	ELSIF (ShowBox3) THEN
		BoxPosition3 := LREAL_TO_REAL(Conveyor.Position - BoxOffset3);
		Box3Transform := '{"select":"#box_red_3","display":true,"duration":2,"scale":[1,1],"translate":[';
		brsftoa(LREAL_TO_REAL(BoxPosition3*100),ADR(PositionString));
		brsstrcat(ADR(Box3Transform),ADR(PositionString));
		brsstrcat(ADR(Box3Transform),ADR(',0]}'));		
	ELSE
		BoxPosition3 := 0;
		Box3Transform := '{"select":"#box_red_3","display":false,"duration":2,"scale":[0.1,0.1],"translate":[30,0]}';
	END_IF
	IF (BoxPosition3 > EXIT_POSITION) THEN
		ShowBox3 := FALSE;
		redCaseComplete := TRUE;
	END_IF
	
	// Handling the light
	IF (WorklightOn) THEN
		WorklightTransform := '{"select":"#work_light_beam","display":true,"duration":2,"translate":[0,0]}';
	ELSE
		WorklightTransform := '{"select":"#work_light_beam","display":false,"duration":2,"translate":[0,0]}';
	END_IF
		
	// Creating the transform string from all elements
	TotalTransform := '[';
	brsstrcat(ADR(TotalTransform),ADR(WorklightTransform));
	brsstrcat(ADR(TotalTransform),ADR(','));
	brsstrcat(ADR(TotalTransform),ADR(Box1Transform));
	brsstrcat(ADR(TotalTransform),ADR(','));
	brsstrcat(ADR(TotalTransform),ADR(Box2Transform));
	brsstrcat(ADR(TotalTransform),ADR(','));
	brsstrcat(ADR(TotalTransform),ADR(Box3Transform));
	brsstrcat(ADR(TotalTransform),ADR(','));
	brsstrcat(ADR(TotalTransform),ADR(Motor1Transform));
	brsstrcat(ADR(TotalTransform),ADR(','));
	brsstrcat(ADR(TotalTransform),ADR(Motor2Transform));
	brsstrcat(ADR(TotalTransform),ADR(','));
	brsstrcat(ADR(TotalTransform),ADR(Motor3Transform));
	brsstrcat(ADR(TotalTransform),ADR(','));
	brsstrcat(ADR(TotalTransform),ADR(Motor4Transform));
	brsstrcat(ADR(TotalTransform),ADR(']'));	
		
	// Calling the FBs
	Conveyor();
	TON_ShowBox1();
	TON_ShowBox2();
	TON_ShowBox3();
	 
END_PROGRAM

PROGRAM _EXIT
	(* Insert code here *)
	 
END_PROGRAM

