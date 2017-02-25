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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;

import org.usfirst.frc.team3223.enums.AutonomousMode;
import org.usfirst.frc.team3223.enums.DriveState;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
	Command autonomousCommand;
	NetworkTable networkTable;
	SensorManager sensorManager;
	TurningStateMachine turningStateMachine;
	TranslationalStateMachine translationalStateMachine;
	JoystickManager joystickManager;
	
	private Servo structurePosition;
	private boolean isAuto;
	private boolean isHighGoalPosition;
	private boolean isGearPosition;

	private DriveState mode = DriveState.HumanDrive;
	private AutonomousMode autoMode;
	boolean dPad = false;
	private Joystick[] pilots = new Joystick[2];
	private int currPilot = 0;
	private int rumbleCount;
	private int FarGearState;
	private double shooterSpeed = .75;
	private double intakeSpeed = .8;

	private static final int F_L_PORT = 6, F_R_PORT = 4, B_L_PORT = 0, B_R_PORT = 3, SHOOT_PORT = 2, ROPE_PORT = 1, INTAKE_PORT = 5, SERVO_PORT = 7; //changed B_L_PORT from 0

	private SpeedController fore_left_motor, fore_right_motor, back_left_motor, back_right_motor, shoot_motor, rope_motor, intake_motor;
	private Encoder encoder;

	private static final int HIGH_MAX_XOFFSET = 160;
	private int highBounds = 3;
	private double highBump = 0.3;
	private double highFactor = .5;
	private boolean seesHighGoal = false;
	
	private static final int LIFT_MAX_ANGLE = 90;
	private double angleBounds = 5;
	private double angleBump = .1;
	private double angleFactor = .21;
	private static final int LIFT_MAX_XOFFSET = 600;
	private double transBounds = 20;
	private double transBump = .2;
	private double transFactor = .36;
	private static final int LIFT_MAX_ZOFFSET = 1500;
	private boolean seesLift = false;
	
	private double outputRotValue;
	private double outputXTransValue;
	private double outputYTransValue;
	
	double errored = 0;
	private boolean isTurning = false;

	private VisionState visionState;
	private RecorderContext recorderContext;
	private RobotDrive masterDrive;
	
	private long startTime;


	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		// Initialize all subsystems
		pilots[0] = new Joystick(0);// 0 is joystick import port on driver panel
		pilots[1] = new Joystick(1);
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
		recorderContext.add("errored", () -> errored);
		recorderContext.add("auto Mode", () -> autoModeToInt());
		recorderContext.add("FarGearState", () -> FarGearState);
		recorderContext.add("heading", () -> sensorManager.getAngle());
		recorderContext.add("fore_left_voltage", () -> fore_left_motor.get());
		recorderContext.add("fore_right_voltage", () -> fore_right_motor.get());
		recorderContext.add("back_left_voltage", () -> back_left_motor.get());
		recorderContext.add("back_right_voltage", () -> back_right_motor.get());
		recorderContext.add("servo angle", () -> structurePosition.getAngle());
		recorderContext.add("X Acceleration", () -> sensorManager.getXAccel());
		recorderContext.add("Y Acceleration", () -> sensorManager.getYAccel());
		recorderContext.add("Z Acceleration", () -> sensorManager.getZAccel());
		

		fore_left_motor = new TalonSRX(F_L_PORT);
		fore_right_motor = new TalonSRX(F_R_PORT);
		back_left_motor = new TalonSRX(B_L_PORT);
		back_right_motor = new TalonSRX(B_R_PORT);

		fore_right_motor.setInverted(true);//switch from clockwise to front-back
		back_right_motor.setInverted(true);

		masterDrive = new RobotDrive(fore_left_motor, back_left_motor, fore_right_motor, back_right_motor);

		shoot_motor = new Talon(SHOOT_PORT);
		rope_motor = new Talon(ROPE_PORT);
		intake_motor = new Talon(INTAKE_PORT);

		networkTable = NetworkTable.getTable("SmartDashboard");
		networkTable.addTableListener(this);

		joystickManager = new JoystickManager(()-> activeJoystick());
		visionState = new VisionState();
		sensorManager = new SensorManager();
		structurePosition = new Servo(SERVO_PORT);
		structurePosition.setAngle(10);
		isHighGoalPosition = false;
		isGearPosition = true;
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
		
		SmartDashboard.putString("DB/String 5", ""+angleBounds);
		SmartDashboard.putString("DB/String 6", ""+angleBump);
		SmartDashboard.putString("DB/String 7", ""+angleFactor);
		
		SmartDashboard.putNumber("DB/Slider 0", transBounds);
		SmartDashboard.putNumber("DB/Slider 1", transBump);
		SmartDashboard.putNumber("DB/Slider 2", transFactor);
		SmartDashboard.putNumber("DB/Slider 3", shooterSpeed);
	}

	public void switchSensorPosition(){
		
	}
	
	public void teleopInit() {
		isAuto = false;
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		getInput();
		switch (mode) {
		case HumanDrive:
			if (activeJoystick().getRawButton(4)){
				swapActiveJoystick();
			}
			driveHuman();
			shoot();
			intake();
			climb();
			if(activeJoystick().getRawButton(10)) {
				mode = DriveState.TurnLikeItsTuesday;
			}
			break;
		case FindHighGoal:
			findHighGoal();
			break;
		case FindLiftStart:
			findLift();
			break;
		case FindLiftContinue:
			goLift();
			break;
		case TurnLikeItsTuesday:
			turn();
			break;
		}
		
		joystickManager.tick();
		sensorManager.tick();
		
		networkTable.putNumber("encoder", encoder.getDistance());

		// SmartDashboard.putString("DB/String 5","Raw
		// Count:"+encoder.getRaw());
		// SmartDashboard.putString("DB/String 6", "Count:" + encoder.get());
		// SmartDashboard.putString("DB/String 7", "Rate:" + encoder.getRate());
		networkTable.putNumber("Raw Count", encoder.getRaw());
		networkTable.putNumber("Count", encoder.get());
		networkTable.putNumber("Rate", encoder.getRate());
		networkTable.putBoolean("Left DPAD", joystickManager.isLeftDPAD());
		networkTable.putBoolean("Right DPAD", joystickManager.isRightDPAD());
		networkTable.putBoolean("A button", activeJoystick().getRawButton(1));
		networkTable.putBoolean("B button", activeJoystick().getRawButton(2));
		networkTable.putBoolean("X button", activeJoystick().getRawButton(3));
		networkTable.putBoolean("Y button", activeJoystick().getRawButton(4));
		networkTable.putBoolean("Right Bumper", activeJoystick().getRawButton(6));
		networkTable.putBoolean("Left Bumper", activeJoystick().getRawButton(5));

		networkTable.putBoolean("Sees Lift", visionState.seesLift());
		networkTable.putNumber("xOffset Lift", visionState.getxOffsetLift());
		networkTable.putBoolean("Sees High Goal", visionState.seesHighGoal());
		networkTable.putNumber("psi Lift", visionState.getPsiLift());
		networkTable.putNumber("servoAngle", structurePosition.getAngle());
		networkTable.putBoolean("isHighGoalPosition", isHighGoalPosition);
		networkTable.putBoolean("isGearPosition", isGearPosition);
		networkTable.putNumber("XAccel", sensorManager.XAccel);
		networkTable.putNumber("YAccel", sensorManager.YAccel);
		networkTable.putNumber("ZAccel", sensorManager.ZAccel);
		recorderContext.tick();
		
	}

	private void getInput() {
		if (activeJoystick().getRawButton(1)) {//A
			mode = DriveState.HumanDrive;
		}
		if (activeJoystick().getRawButton(2)) {//B
			mode = DriveState.FindHighGoal;
		}
		if (activeJoystick().getRawButton(3)) {//X
			mode = DriveState.FindLiftStart;
		}
		SmartDashboard.putString("DB/String 3", "Mode:" + mode);
		seesHighGoal = visionState.seesHighGoal();
		SmartDashboard.putString("DB/String 4", "TP:" + seesHighGoal);
		seesLift = visionState.seesLift();
		SmartDashboard.putBoolean("DB/Button 3", seesLift);
	}
	
	private Joystick activeJoystick() {
		return pilots[currPilot];
	}

	private void swapActiveJoystick() {
		if (rumbleCount == 0) {
			pilots[(currPilot + 1) % 2].setRumble(GenericHID.RumbleType.kLeftRumble, 0);
			pilots[(currPilot + 1) % 2].setRumble(GenericHID.RumbleType.kRightRumble, 0);
		}
		if (rumbleCount < 0) {
			if (pilots[(currPilot + 1) % 2].getRawButton(4))// if not-pilot push 'y'
			{
				pilots[currPilot].setRumble(GenericHID.RumbleType.kLeftRumble, 1);
				pilots[currPilot].setRumble(GenericHID.RumbleType.kRightRumble, 1);
				currPilot = (currPilot + 1) % 2;// switch pilot
				rumbleCount = 10;
			}
		} else {
			rumbleCount--;
		}
	}

	private void findHighGoal() {
		double rotationalValue;

		highBounds = (int) SmartDashboard.getNumber("DB/Slider 0", highBounds);
		highBump = SmartDashboard.getNumber("DB/Slider 1", highBump);
		highFactor = SmartDashboard.getNumber("DB/Slider 2", highFactor);
		
		if(!isHighGoalPosition && isGearPosition){
			structurePosition.setAngle(145);
			isGearPosition = false;
			startTime = System.currentTimeMillis();
		}
		
		if(!isHighGoalPosition && !isGearPosition){
			if(System.currentTimeMillis()-startTime>=700){
				isHighGoalPosition = true;
			}
		}
		

		if (seesHighGoal&&isHighGoalPosition) {
			double pixels = visionState.getxPixelOffsetHighGoal();
			if (pixels < highBounds * -1 || pixels > highBounds) {
				rotationalValue = ((pixels / HIGH_MAX_XOFFSET) * highFactor);// Adjustable
				if (rotationalValue > 0) {
					rotationalValue += highBump;// get over hump
				} else {
					rotationalValue -= highBump;
				}

				driveRobot(0, 0, rotationalValue);

				SmartDashboard.putString("DB/String 2", "RV=" + rotationalValue);
				SmartDashboard.putString("DB/String 1", "PX=" + visionState.getxPixelOffsetHighGoal());
			} else {
				driveRobot(0, 0, 0);
				mode = DriveState.HumanDrive;
				// perform high goal
				// return control to teleop
			}
		} else if(isHighGoalPosition) {
			driveRobot(0, 0, 0);
			mode = DriveState.HumanDrive;
		}
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
				x = clamp(xOffset, 0, 150, 0.2);
				y = clamp(zOffset, 500, 150, 0.4);
				rot = clamp(theta, 0, Math.toRadians(10), 0.2);
			}else{
				x = clamp(xOffset, 0, 150, 0.2);
				y = clamp(zOffset, 500, 150, 0.2);
				rot = clamp(psi, 0, Math.toRadians(10), 0.2);
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
		
		if(isHighGoalPosition && !isGearPosition){
			structurePosition.setAngle(10);
			isHighGoalPosition = false;
			startTime = System.currentTimeMillis();
		}
		
		if(!isHighGoalPosition && !isGearPosition){
			if(System.currentTimeMillis()-startTime>=700){
				isGearPosition = true;
			}
		}
		
		if (seesLift&&isGearPosition) {
			double xOffset = visionState.getxOffsetLift() + 200;// mm
			double psiAngle = Math.toDegrees(visionState.getPsiLift());// rad ->
																		// Degree
			SmartDashboard.putString("DB/String 1", "xOff:" + xOffset);
			SmartDashboard.putString("DB/String 2", "psi:" + psiAngle);

			angleBounds = Double.parseDouble(SmartDashboard.getString("DB/String 5", "10"));
			angleBump = Double.parseDouble(SmartDashboard.getString("DB/String 6", ".1"));
			angleFactor = Double.parseDouble(SmartDashboard.getString("DB/String 7", ".4"));

			transBounds = SmartDashboard.getNumber("DB/Slider 0", 10);
			transBump = SmartDashboard.getNumber("DB/Slider 1", .4);
			transFactor = SmartDashboard.getNumber("DB/Slider 2", .1);

			double rotVal = 0;
			double transVal = 0;

			if (seesLift) {
				if (psiAngle > angleBounds || psiAngle < -1 * angleBounds)//rotational out of bounds
				{
					rotVal = psiAngle / LIFT_MAX_ANGLE * angleFactor;
					if (rotVal > 0)
						rotVal += angleBump;
					else
						rotVal -= angleBump;
					outputRotValue = rotVal;
				}
				if (xOffset > transBounds || transBounds < -1*xOffset) //out of bounds
				{
					transVal = xOffset / LIFT_MAX_XOFFSET * transFactor;
					if (transVal > 0)
						transVal += transBump;
					else
						transVal -= transBump;
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
		else if(isGearPosition) {
			driveRobot(0, 0, 0);
			if(!isAuto)
				mode = DriveState.HumanDrive;
		}
	}

	private void goLift() {
		driveRobot(0, 0.4, 0);
		if(sensorManager.getYAccel()>1){
			driveRobot(0,0,0);
			if(!isAuto){
				mode = DriveState.HumanDrive;
			}else{
				autoMode = AutonomousMode.Finished;
			}
		}
	}
	
	private void shoot() {
		//shooterSpeed = SmartDashboard.getNumber("DB/Slider 3", 0.0);

		if (activeJoystick().getPOV(0) == 0) {
			shoot_motor.set(shooterSpeed);
		}
		if (activeJoystick().getPOV(0) == 180) {
			shoot_motor.set(0);
		}
	}

	private void intake(){
		if(joystickManager.isIntakeToggled()){
			if(joystickManager.isInverseIntakeToggled()){
				intake_motor.set(-1*intakeSpeed);
			}else{
				intake_motor.set(intakeSpeed);
			} 
		}else{
			intake_motor.set(0);
		}
		
	}
	
	private void climb(){
		if(activeJoystick().getRawButton(7)){
			rope_motor.set(-.5);
		}
		else
		{
			if(activeJoystick().getRawButton(8))
				rope_motor.set(.5);
			else
			rope_motor.set(0);
		}
	}
	
	private void driveHuman() {
		double x = 0;
		double y = 0;
		double rotation = 0;
		
		x = activeJoystick().getRawAxis(0);// x of l stick
		y = activeJoystick().getRawAxis(1);// y of l stick
		if (Math.abs(x) <= .1)
			x = 0;
		if (Math.abs(y) <= .1)
			y = 0;
		
		if (activeJoystick().getRawButton(5))
			x = -1;
		if(activeJoystick().getRawButton(6))
			x = 1;
		
		rotation = activeJoystick().getRawAxis(3) - activeJoystick().getRawAxis(2); // triggers:(right-left)turn
		if (Math.abs(rotation) <= .1)
			rotation = 0;
		
		double angle = 0;
		masterDrive.mecanumDrive_Cartesian(x , y , (rotation) / 1.5, angle);
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
		FarGearState = 1;
		autoMode = AutonomousMode.Selecting;
		isAuto = true;
	}

	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putString("DB/String 0", "State:" + FarGearState);
		SmartDashboard.putString("DB/String 1", "Sees Lift:" + seesLift);
		seesLift = visionState.seesLift();
		intake_motor.set(0);
		rope_motor.set(0);
		shoot_motor.set(0);
		switch (autoMode) {
		case Selecting:
			translationalStateMachine.setInputDistance(15);
			translationalStateMachine.run();
			if(translationalStateMachine.isDone()){
				translationalStateMachine.reset();
				selectGearTarget();
			}
			break;
		case MiddleGear:
			autoMode = AutonomousMode.FindLift;
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
		case Boiler:
			break;
		case Finished:
			driveRobot(0, 0, 0);
			break;
		}
		recorderContext.tick();
		sensorManager.tick();
		networkTable.putBoolean("Sees Lift", visionState.seesLift());
		networkTable.putBoolean("isHighGoalPosition", isHighGoalPosition);
		networkTable.putBoolean("isGearPosition", isGearPosition);
	}
	
	//selects either middle lift or far lift
	private void selectGearTarget(){
		if(visionState.seesLift()&&Math.abs(visionState.getxOffsetLift())<500)
		{
			autoMode = AutonomousMode.MiddleGear;
		}
		else
		{
			autoMode = AutonomousMode.FarGear;
		}
	}
	
	//figures out which side we are on, if it cannot find a lift, goes forward and returns control to human
	private void approachFarGear()
	{
		// Turn right 30 degrees
		if(FarGearState == 1){
			turningStateMachine.setInputAngle(Math.toRadians(45));
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
			turningStateMachine.setInputAngle(Math.toRadians(-90));
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
			turningStateMachine.setInputAngle(Math.toRadians(45));
			turningStateMachine.run();
			if(turningStateMachine.isDone()){
				FarGearState = 6;
			}
		}
		
		//move forward and cross baseline
		if(FarGearState == 6){
			translationalStateMachine.setInputDistance(10);
			translationalStateMachine.run();
			if(translationalStateMachine.isDone()){
				FarGearState = 7;
				translationalStateMachine.reset();
				driveRobot(0, 0, 0);
				mode = DriveState.HumanDrive;
			}
		}
		
	}
	
	public int autoModeToInt(){
		if(autoMode == null) return 7;
		switch (autoMode) {
		case Selecting:
			return 0;
		case MiddleGear:
			return 1;
		case FarGear:
			return 2;
		case FindLift:
			return 3;
		case GoLift:
			return 4;
		case Boiler:
			return 5;
		case Finished:
			return 6;
		}
		return 7;
	}
	
	public void valueChanged(ITable source, String key, Object value, boolean isNew) {
	}
	/*
	 * Buttons: 
	 * A:human drive
	 * B:find High goal
	 * X:find lift 
	 * Y:Swap Pilots 
	 * RB:strafe-Right 
	 * LB:strafe-Left 
	 * SEL:climber
	 * STA:straight back
	 * R3:
	 * L3:
	 * 
	 * Axis:
	 * RT:rotate right 
	 * LT:rotate left 
	 * RS: 
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
