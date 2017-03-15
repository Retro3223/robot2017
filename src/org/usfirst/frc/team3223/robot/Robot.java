/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;

import org.usfirst.frc.team3223.enums.AutonomousMode;
import org.usfirst.frc.team3223.enums.DriveState;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements ITableListener {
	NetworkTable networkTable;
	SensorManager sensorManager;
	TurningStateMachine turningStateMachine;
	TranslationalStateMachine translationalStateMachine;
	JoystickManager joystickManager;

	private boolean isAuto;

	private DriveState mode = DriveState.HumanDrive;
	private AutonomousMode autoMode;
	private AutonomousMode nextAutoMode = AutonomousMode.MiddleGear;
	private String selectedAutoMode = "none";
    private int autoBegin = 0;
	
	private int FarGearState;

	private static final int F_L_PORT = 2;
	private static final int F_R_PORT = 3;
	private static final int B_L_PORT = 0;
	private static final int B_R_PORT = 1;
	private static final int ROPE_PORT = 4;

	private SpeedController fore_left_motor, fore_right_motor, back_left_motor, back_right_motor, shoot_motor, rope_motor, intake_motor;
	private Encoder encoder;
	
	private static final int LIFT_MAX_ANGLE = 90;//will not change

    // don't try to correct for angle; lower this to reenable
	private double angleBounds = 50;//degrees- happy zone
	private double angleBump = .1;//motor val added to overcome friction [-1,1]
	private double angleFactor = .21;//some random constant that Rhys set or the slope of power line
	private static final int LIFT_MAX_XOFFSET = 600;//mm shouldn't change
	private double transBounds = 20;//mm- happy zone
	private double transBump = .25;//power added to overcome friction [-1,1]
	private double transFactor = .36;//some random constant - slope of power line
	
	private boolean seesLift = false;
	
	private double outputRotValue;
	private double outputXTransValue;
	private double outputYTransValue;
	
	private boolean isTurning = false;

	private VisionState visionState;
	private RecorderContext recorderContext;
	private RobotDrive masterDrive;
	
	private long startTime;
	private int forwardLittleTimer;


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		// Initialize all subsystems
		

        this.setupLogger();
				
		fore_left_motor = new TalonSRX(F_L_PORT);
		fore_right_motor = new TalonSRX(F_R_PORT);
		back_left_motor = new TalonSRX(B_L_PORT);
		back_right_motor = new TalonSRX(B_R_PORT);
		
		fore_right_motor.setInverted(true);//switch from clockwise to front-back
		back_right_motor.setInverted(true);

		masterDrive = new RobotDrive(fore_left_motor, back_left_motor, fore_right_motor, back_right_motor);

		rope_motor = new Talon(ROPE_PORT);

		networkTable = NetworkTable.getTable("SmartDashboard");
		networkTable.addTableListener(this, true);

		joystickManager = new JoystickManager();
		visionState = new VisionState();
		sensorManager = new SensorManager();
		FarGearState = 1;
		turningStateMachine = new TurningStateMachine(visionState, sensorManager, (Double voltage) -> {
			double v = voltage.doubleValue();
			fore_left_motor.set(v);
			back_left_motor.set(v);
			fore_right_motor.set(-v);
			back_right_motor.set(-v);
		});
		translationalStateMachine = new TranslationalStateMachine(this);
		this.encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		
		// instantiate the command used for the autonomous period

		// Show what command your subsystem is running on the SmartDashboard
		// SmartDashboard.putData(drivetrain);
		
		/*SmartDashboard.putString("DB/String 5", ""+angleBounds);
		SmartDashboard.putString("DB/String 6", ""+angleBump);
		SmartDashboard.putString("DB/String 7", ""+angleFactor);
		
		SmartDashboard.putNumber("DB/Slider 0", transBounds);
		SmartDashboard.putNumber("DB/Slider 1", transBump);
		SmartDashboard.putNumber("DB/Slider 2", transFactor);
		SmartDashboard.putNumber("DB/Slider 3", shooterSpeed);*/
	}

    private void setupLogger() {
		recorderContext = new RecorderContext("lift");
		recorderContext.add("seesLift", () -> seesLift ? 1 : 0);
		recorderContext.add("mode", () -> mode.ordinal());
		recorderContext.add("rotVal", () -> outputRotValue);
		recorderContext.add("xtransVal", () -> outputXTransValue);
		recorderContext.add("ytransVal", () -> outputYTransValue);
		recorderContext.add("xOffset", () -> visionState.getxOffsetLift());
		recorderContext.add("zOffset", () -> visionState.getzOffsetLift());
		recorderContext.add("theta", () -> visionState.getThetaLift());
		recorderContext.add("psi", () -> visionState.getPsiLift());
		recorderContext.add("angleBounds", () -> angleBounds);
		recorderContext.add("angleFactor", () -> angleFactor);
		recorderContext.add("auto Mode", () -> autoMode == null ? 7 : autoMode.ordinal());
		recorderContext.add("FarGearState", () -> FarGearState);
		recorderContext.add("heading", () -> sensorManager.getAngle());
		recorderContext.add("fore_left_voltage", () -> fore_left_motor.get());
		recorderContext.add("fore_right_voltage", () -> fore_right_motor.get());
		recorderContext.add("back_left_voltage", () -> back_left_motor.get());
		recorderContext.add("back_right_voltage", () -> back_right_motor.get());
		recorderContext.add("X Acceleration", () -> sensorManager.getXAccel());
		recorderContext.add("Y Acceleration", () -> sensorManager.getYAccel());
		recorderContext.add("Z Acceleration", () -> sensorManager.getZAccel());
    }

	
	public void teleopInit() {
		isAuto = false;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		joystickManager.tick();
		getInput();
		switch (mode) {
		case HumanDrive:
			joystickManager.swapActiveJoystick();
			driveHuman();
			climb();
			break;
		case FindLiftStart:
			findLift();
			break;
		case FindLiftContinue:
			goLift();
			break;
		}
		
		sensorManager.tick();
		
		networkTable.putNumber("encoder", encoder.getDistance());

		// SmartDashboard.putString("DB/String 5","Raw
		// Count:"+encoder.getRaw());
		// SmartDashboard.putString("DB/String 6", "Count:" + encoder.get());
		// SmartDashboard.putString("DB/String 7", "Rate:" + encoder.getRate());
		networkTable.putNumber("Raw Count", encoder.getRaw());
		networkTable.putNumber("Count", encoder.get());
		networkTable.putNumber("Rate", encoder.getRate());
		networkTable.putBoolean("Sees Lift", visionState.seesLift());
		networkTable.putNumber("xOffset Lift", visionState.getxOffsetLift());
		networkTable.putBoolean("Sees High Goal", visionState.seesHighGoal());
		networkTable.putNumber("psi Lift", visionState.getPsiLift());
		networkTable.putNumber("XAccel", sensorManager.XAccel);
		networkTable.putNumber("YAccel", sensorManager.YAccel);
		networkTable.putNumber("ZAccel", sensorManager.ZAccel);
		recorderContext.tick();
		
	}
	

	private void getInput() {
		if (joystickManager.humanDriveButtonPressed()) {
			mode = DriveState.HumanDrive;
		}
		if (joystickManager.findLiftButtonPressed()) {
			mode = DriveState.FindLiftStart;
		}
		SmartDashboard.putString("DB/String 3", "Mode:" + mode);
		seesLift = visionState.seesLift();
		SmartDashboard.putBoolean("DB/Button 3", seesLift);
	}

	private void positionLift() {
		double x = 0;
		double y = 0;
		double rot = 0;
		
		if(seesLift) {
			double xOffset = visionState.getxOffsetLift();
			double zOffset = visionState.getzOffsetLift();
			double theta = visionState.getThetaLift();
			double psi = visionState.getPsiLift();
			
			if(zOffset > 2000) {
				x = clamp(xOffset, 0, 150, 0.3);
				y = clamp(zOffset, 500, 150, 0.6);
				
			}else{
				x = clamp(xOffset, 0, 150, 0.3);
				y = clamp(zOffset, 500, 150, 0.4);
				
			}
		}
		
		masterDrive.mecanumDrive_Cartesian(x, y, rot, 0);
	}
	
	/**
	 * given one of the measured offsets (x, y, theta, psi), choose the output to give to that component of 
	 * mechanum drive to bring measured closer to desired, if necessary.
	 * @param input measured value from vision
	 * @param desired what we want input to be
	 * @param range |max acceptable input - min acceptable input|
	 * @param absOutput output, if there should be any, excluding sign.
	 * @return
	 */
	private double clamp(double input, double desired, double range, double absOutput) {
		double output = 0;
		if(desired - 0.5 * range >= input) {
			output = absOutput;
		}else if(desired + 0.5 * range <= input) {
			output = -absOutput;
		}
		return output;
	}
	
	private void findLift() {
		
		if (seesLift) {
			double xOffset = visionState.getxOffsetLift() + -10f;// mm TODO xOffset on actual robot
			double psiAngle = Math.toDegrees(visionState.getPsiLift());// rad ->
																		// Degree
			SmartDashboard.putString("DB/String 1", "xOff:" + xOffset);
			SmartDashboard.putString("DB/String 2", "psi:" + psiAngle);
			
			/*angleBounds = Double.parseDouble(SmartDashboard.getString("DB/String 5", "10"));
			angleBump = Double.parseDouble(SmartDashboard.getString("DB/String 6", ".1"));
			angleFactor = Double.parseDouble(SmartDashboard.getString("DB/String 7", ".4"));

			transBounds = SmartDashboard.getNumber("DB/Slider 0", 10);
			transBump = SmartDashboard.getNumber("DB/Slider 1", .4);
			transFactor = SmartDashboard.getNumber("DB/Slider 2", .1);*/

			double rotVal = 0;
			double transVal = 0;

			if (seesLift) {
				/*if (psiAngle > angleBounds || psiAngle < -1 * angleBounds)//rotational out of bounds
				{
					rotVal = psiAngle / LIFT_MAX_ANGLE * angleFactor;
					if (rotVal > 0)
						rotVal += angleBump;
					else
						rotVal -= angleBump;
					outputRotValue = rotVal;
				}*/
				if (xOffset > transBounds || transBounds < -1*xOffset) //out of bounds
				{	
					if(xOffset>transBounds){
						transVal = -0.3;
					}else{
						transVal = 0.3;
					}
					/*transVal = xOffset / LIFT_MAX_XOFFSET * transFactor;
					if (transVal > 0)
						transVal += transBump;
					else
						transVal -= transBump;*/
					outputXTransValue = transVal;
				}
			
				SmartDashboard.putString("DB/String 8", "" + rotVal);
				SmartDashboard.putNumber("DB/Slider 3", transVal);
				
				if (rotVal == 0 && transVal == 0)//both happy - lined up
				{
					if(isAuto)
						
						autoMode = AutonomousMode.GoLift;
					else
						mode = DriveState.FindLiftContinue;
				} 
				else {
					driveRobot(transVal, 0, rotVal);
				}
			}
		} 
		else{
			driveRobot(0, 0, 0);
			if(!isAuto)
				mode = DriveState.HumanDrive;
		}
	}

	private void goLift() {
		driveRobot(0, -0.3, 0);
		if(sensorManager.getYAccel()>1){
			driveRobot(0,0,0);
			if(!isAuto){
				mode = DriveState.HumanDrive;
			}else{
				autoMode = AutonomousMode.GoForwardALittle;
			}
		}
	}

	
	private void climb(){
        if(joystickManager.isClimberButtonToggled()) {
			rope_motor.set(-1);
		} else if(joystickManager.isReverseClimberButtonDepressed()) {
            rope_motor.set(1);
        } else {
            rope_motor.set(0);
		}
	}
	
	private void driveHuman() {
		double x = 0;
		double y = 0;
		double rotation = 0;
		
		x = joystickManager.activeJoystick().getRawAxis(0);// x of l stick
		y = joystickManager.activeJoystick().getRawAxis(1);// y of l stick
		if (Math.abs(x) <= .15) {
			x = 0;
        }

		if (Math.abs(y) <= .15) {
			y = 0;
        }
		
		double rightX = joystickManager.activeJoystick().getRawAxis(4);
		double rightY = joystickManager.activeJoystick().getRawAxis(5);
		if (Math.abs(rightX) > .15 || Math.abs(rightY) > .15) {
			y = rightY / 3;
			x = rightX / 3;
		}
		
		rotation = joystickManager.activeJoystick().getRawAxis(3) - joystickManager.activeJoystick().getRawAxis(2); // triggers:(right-left)turn
		if (Math.abs(rotation) <= .1) {
			rotation = 0;
        }
		
		masterDrive.mecanumDrive_Cartesian(x , y , (rotation) / 1.4, 0);
	}

	public void driveRobot(double x, double y, double rotation) {
		masterDrive.mecanumDrive_Cartesian(x, y, rotation * -1, 0);
	}

	public void turn() {
		if (!isTurning) {
			isTurning = true;
			turningStateMachine.reset();
			turningStateMachine.setInputAngle(Math.toRadians(30));
		}

		if (turningStateMachine.isDone()) {
			mode = DriveState.HumanDrive;
		} else {
			turningStateMachine.run();
		}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {

	}

	@Override
	public void autonomousInit() {
        networkTable.putNumber("autonomousBegin", autoBegin++);
		FarGearState = 1;
		autoMode = AutonomousMode.DashboardSelecting;
        startTime = System.currentTimeMillis();
		isAuto = true;
		forwardLittleTimer = 20;
	}

	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putString("DB/String 0", "State:" + FarGearState);
		SmartDashboard.putString("DB/String 1", "Sees Lift:" + seesLift);
		seesLift = visionState.seesLift();
		//intake_motor.set(0);
		rope_motor.set(0);
		//shoot_motor.set(0);
		switch (autoMode) {
		case Selecting:
			translationalStateMachine.setInputDistance(15);
			translationalStateMachine.run();
			if(translationalStateMachine.isDone()){
				translationalStateMachine.reset();
				selectGearTarget();
			}
			break;
        case DashboardSelecting:
            // don't know how long it will take networktables to update
            // robot's version of data, but its a potential race condition.
            // let's just wait 0.1 seconds and hope that's good enough
            if(System.currentTimeMillis() - startTime > 100) {
                selectDashboardMode();
            }
            break;
		case Forward:
			translationalStateMachine.setInputDistance(25);
			translationalStateMachine.run();
			if(translationalStateMachine.isDone()){
				translationalStateMachine.reset();
				autoMode = nextAutoMode;
			}
			break;
		case MiddleGear:
			translationalStateMachine.setInputDistance(20);
			translationalStateMachine.run();
			if(translationalStateMachine.isDone()){
				translationalStateMachine.reset();
				autoMode = AutonomousMode.FindLift;
			}
			break;
        case LeftFarGear:
			approachLeftFarGear();
            break;
        case RightFarGear:
			approachRightFarGear();
            break;
		case FarGear:
			approachFarGear();
			break;
		case FindLift:
			findLift();
			break;
		case GoLift:
			goLift();
			break;
		case Finished:
			driveRobot(0, 0, 0);
			break;
		case GoForwardALittle:
			if(forwardLittleTimer >= 0)
			{
				forwardLittleTimer--;
			driveRobot(0,0.2,0);
			}
			else
			{
				autoMode = AutonomousMode.Finished;
			}
			break;
		}
		recorderContext.tick();
		sensorManager.tick();
		networkTable.putBoolean("Sees Lift", visionState.seesLift());
	}
	
	//selects either middle lift or far lift
	private void selectGearTarget(){
		if(visionState.seesLift()&&Math.abs(visionState.getxOffsetLift())<200)
		{
			autoMode = AutonomousMode.MiddleGear;
		}
		else
		{
			autoMode = AutonomousMode.FarGear;
		}
	}

    private void selectDashboardMode() {
        switch(selectedAutoMode) {
            case "middleGear":
                autoMode = AutonomousMode.MiddleGear;
                nextAutoMode = AutonomousMode.MiddleGear;
                break;
            case "leftGear":
                autoMode = AutonomousMode.Forward;
                nextAutoMode = AutonomousMode.LeftFarGear;
                break;
            case "rightGear":
                autoMode = AutonomousMode.Forward;
                nextAutoMode = AutonomousMode.RightFarGear;
                break;
            default:
                autoMode = AutonomousMode.Finished;
        }
    }
	
	//figures out which side we are on, if it cannot find a lift, goes forward and returns control to human
	private void approachFarGear()
	{
		// Turn right 30 degrees
		if(FarGearState == 1){
			turningStateMachine.setInputAngle(Math.toRadians(30));
			turningStateMachine.run();
			if(turningStateMachine.isDone()){
				turningStateMachine.reset();
				FarGearState = 2;
			}
			
		}
		// Check to see Lift, if not switch to turn other way
		if(visionState.seesLift()&&FarGearState == 2){
			autoMode = AutonomousMode.FindLift;
		}else if(!visionState.seesLift()&&FarGearState == 2){
			FarGearState = 3;
		}	
		
		// turn left 60 degrees
		if(FarGearState == 3){
			turningStateMachine.setInputAngle(Math.toRadians(-60));
			turningStateMachine.run();
			if(turningStateMachine.isDone()){
				turningStateMachine.reset();
				FarGearState = 4;
			}
		}
			
		//Check to see lift, if not cross baseline
		if(visionState.seesLift() && FarGearState==4){
				autoMode = AutonomousMode.FindLift;
		} else if(!visionState.seesLift()&&FarGearState == 4){
			FarGearState = 5;
		}
		
		//Turns robot to face straight ahead
		if(FarGearState==5){
			turningStateMachine.setInputAngle(Math.toRadians(30));
			turningStateMachine.run();
			if(turningStateMachine.isDone()){
				FarGearState = 6;
			}
		}
		
		//move forward and cross baseline
		if(FarGearState == 6){
			translationalStateMachine.setInputDistance(15);
			translationalStateMachine.run();
			if(translationalStateMachine.isDone()){
				FarGearState = 7;
				translationalStateMachine.reset();
				driveRobot(0, 0, 0);
				mode = DriveState.HumanDrive;
			}
		}
		
	}

    private void turnLeft(double angle, int turnState, int finishState) {
        angle = -Math.abs(angle);
		if(FarGearState == turnState){
			turningStateMachine.setInputAngle(angle);
			turningStateMachine.run();
			if(turningStateMachine.isDone()){
				turningStateMachine.reset();
				FarGearState = finishState;
			}
		}
    }

    private void turnRight(double angle, int turnState, int finishState) {
        angle = Math.abs(angle);
		if(FarGearState == turnState){
			turningStateMachine.setInputAngle(angle);
			turningStateMachine.run();
			if(turningStateMachine.isDone()){
				turningStateMachine.reset();
				FarGearState = finishState;
			}
		}
    }

	private void approachLeftFarGear()
	{
        double angle = Math.toRadians(40);
		// Turn right X degrees when FarGearState == 1
        turnRight(angle, 1, 2);

		// Check to see Lift, if not turn to face straight ahead
		if(visionState.seesLift()&&FarGearState == 2){
			autoMode = AutonomousMode.FindLift;
		}else if(!visionState.seesLift()&&FarGearState == 2){
			FarGearState = 5;
		}	
		
		//Turns robot to face straight ahead
        turnLeft(angle, 5, 6);
		
		//move forward and cross baseline
		if(FarGearState == 6){
			translationalStateMachine.setInputDistance(15);
			translationalStateMachine.run();
			if(translationalStateMachine.isDone()){
				FarGearState = 7;
				translationalStateMachine.reset();
				driveRobot(0, 0, 0);
				mode = DriveState.HumanDrive;
			}
		}
		
	}

	private void approachRightFarGear()
	{
        double angle = Math.toRadians(40);
		// Turn left X degrees
        turnLeft(angle, 1, 2);

		// Check to see Lift, if not turn to face straight ahead
		if(visionState.seesLift()&&FarGearState == 2){
			autoMode = AutonomousMode.FindLift;
		}else if(!visionState.seesLift()&&FarGearState == 2){
			FarGearState = 5;
		}	
		
		//Turns robot to face straight ahead
        turnRight(angle, 5, 6);
		
		//move forward and cross baseline
		if(FarGearState == 6){
			translationalStateMachine.setInputDistance(15);
			translationalStateMachine.run();
			if(translationalStateMachine.isDone()){
				FarGearState = 7;
				translationalStateMachine.reset();
				driveRobot(0, 0, 0);
				mode = DriveState.HumanDrive;
			}
		}
	}
	
	public void valueChanged(ITable source, String key, Object value, boolean isNew) {
        if(key.equals("autonomousMode")) {
            selectedAutoMode = (String) value;
        }
	}
	/*
	 * Buttons: 
	 * A:human drive
	 * B:find High goal
	 * X:find lift 
	 * Y:Swap Pilots 
	 * RB:Operate Trapdoor 
	 * LB: 
	 * SEL:climber
	 * STA:straight back
	 * R3:Reverse Controls
	 * L3:
	 * 
	 * Axis:
	 * RT:rotate right 
	 * LT:rotate left 
	 * RS: slow movement
	 * LS:x-z movement
	 * 
	 * DPAD:
	 * UP:shooter on 
	 * DOWN:shooter off
	 * LEFT:intake in
	 * RIGHT:intake out
	 * 
	 * Strings: 
	 * 0: 
	 * 1: PX(High) || xOffset(Lift) 
	 * 2: RV(High) || psiAngle(Lift) 
	 * 3: Mode 
	 * 4: seesHighGoal(High)
	 * 
	 * 5: angleBounds (Lift) 
	 * 6: angleBump(Lift)||encoder.getRaw()
	 * 7: angleFactor(Lift)||encoder.get() 
	 * 8: angleVal(Lift)||encoder.getRate()
	 * 9: zOffset(Lift)
	 * 
	 * Sliders: 
	 * 0: Bounds(High)||transBounds(Lift) 
	 * 1: Bump(High)||transBumb(Lift) 
	 * 2: Factor(High)||Factor(Lift) 
	 * 3: Speed(Shoot)||transVal(Lift)
	 * 
	 * Buttons: 
	 * 0: dPad(Drive) 
	 * 1: 
	 * 2: 
	 * 3: seesLift(Lift)
	 * 
	 * buttons: 
	 * 1-a, 2-b, 3-x, 4-y, 5-lb, 6-rb, 7-back, 8-start, 9-l3, 10-r3
	 * 
	 * public boolean getRawButton(int button) 
	 * 
	 * Axis indexes: 
	 * 0-LeftX, 1-LeftY, 2-Left Trigger (0-1), 3-Right Trigger (0-1), 4-RightX, 5-RightY
	 * 
	 * public double getRawAxis(int axis)
	 * 
	 * public int getPOV(int pov) 
	 *  - for d-pad returns degrees from north, clockwise, -1 if not pressed.
	 * 
	 * SmartDashboard.putString("DB/String 0", "My 21 Char TestString");
	 * dashData = SmartDashboard.getString("DB/String 0", "myDefaultData");
	 * 
	 * SmartDashboard.putBoolean("DB/Button 0", true); (default value of false):
	 * boolean buttonValue = SmartDashboard.getBoolean("DB/Button 0", false);
	 * 
	 * SmartDashboard.putNumber("DB/Slider 0", 2.58); (default value of 0.0):
	 * double dashData = SmartDashboard.getNumber("DB/Slider 0", 0.0);
	 * 
	 * 
	 */
}
