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

import org.usfirst.frc.team3223.enums.AutonomousMode;
import org.usfirst.frc.team3223.enums.DriveState;

import com.kauailabs.navx.frc.AHRS;

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
	JoystickManager joystickManager;

	private DriveState mode = DriveState.HumanDrive;
	private AutonomousMode autoMode;
	boolean dPad = false;
	private Joystick[] pilots = new Joystick[2];
	private int currPilot = 0;
	private int rumbleCount;

	private static final int F_L_PORT = 7, F_R_PORT = 9, B_L_PORT = 6, B_R_PORT = 8, SHOOT_PORT = 4, ROPE_PORT = 5, INTAKE_PORT = 3;
	private SpeedController fore_left_motor, fore_right_motor, back_left_motor, back_right_motor, shoot_motor,
			rope_motor, intake_motor;
	private Encoder encoder;

	private int bounds = 3;
	private double bump = 0.3;
	private double factor = .5;
	private boolean seesHighGoal = false;
	private boolean seesLift = false;
	private double outputRotValue;
	private double outputTransValue;
	double angleBounds = 0;
	double angleFactor = 0;
	double errored = 0;
	private boolean isTurning = false;

	private VisionState visionState;
	private RecorderContext recorderContext;
	private RobotDrive masterDrive;

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
		recorderContext.add("transVal", () -> outputTransValue);
		recorderContext.add("xOffset", () -> visionState.getxOffsetLift());
		recorderContext.add("zOffset", () -> visionState.getzOffsetLift());
		recorderContext.add("theta", () -> visionState.getThetaLift());
		recorderContext.add("psi", () -> visionState.getPsiLift());
		recorderContext.add("angleBounds", () -> angleBounds);
		recorderContext.add("angleFactor", () -> angleFactor);
		recorderContext.add("errored", () -> errored);

		fore_left_motor = new TalonSRX(F_L_PORT);
		fore_right_motor = new TalonSRX(F_R_PORT);
		back_left_motor = new TalonSRX(B_L_PORT);
		back_right_motor = new TalonSRX(B_R_PORT);

		fore_right_motor.setInverted(true);// for whatever reason, right side
											// motors spin wrong way.
		back_right_motor.setInverted(true);// therefore, invert the motors in
											// code.

		masterDrive = new RobotDrive(fore_left_motor, back_left_motor, fore_right_motor, back_right_motor);

		shoot_motor = new Talon(SHOOT_PORT);
		rope_motor = new Talon(ROPE_PORT);
		intake_motor = new Talon(INTAKE_PORT);

		networkTable = NetworkTable.getTable("SmartDashboard");
		networkTable.addTableListener(this);

		joystickManager = new JoystickManager(()-> activeJoystick());
		visionState = new VisionState();
		sensorManager = new SensorManager();
		turningStateMachine = new TurningStateMachine(visionState, sensorManager, (Double voltage) -> {
			double v = voltage.doubleValue();
			fore_left_motor.set(v);
			back_left_motor.set(v);
			fore_right_motor.set(-v);
			back_right_motor.set(-v);
		});
		this.encoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		// instantiate the command used for the autonomous period

		// Show what command your subsystem is running on the SmartDashboard
		// SmartDashboard.putData(drivetrain);

	}

	public void teleopInit() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		getInput();
		switch (mode) {
		case HumanDrive:
			swapActiveJoystick();
			driveHuman();
			shoot();
			intake();
			climb();
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
		

		// SmartDashboard.putString("DB/String 5","Raw
		// Count:"+encoder.getRaw());
		// SmartDashboard.putString("DB/String 6", "Count:" + encoder.get());
		// SmartDashboard.putString("DB/String 7", "Rate:" + encoder.getRate());
		networkTable.putNumber("Raw Count", encoder.getRaw());
		networkTable.putNumber("Count", encoder.get());
		networkTable.putNumber("Rate", encoder.getRate());
		recorderContext.tick();

	}

	private void getInput() {
		if (activeJoystick().getRawButton(1)) {
			mode = DriveState.HumanDrive;
		}
		if (activeJoystick().getRawButton(2)) {
			mode = DriveState.FindHighGoal;
		}
		if (activeJoystick().getRawButton(3)) {
			mode = DriveState.FindLiftStart;
		}
		SmartDashboard.putString("DB/String 3", "Mode:" + mode);
		seesHighGoal = visionState.seesHighGoal();
		SmartDashboard.putString("DB/String 4", "TP:" + seesHighGoal);
		seesLift = visionState.seesLift();
		SmartDashboard.putBoolean("DB/Button 3", seesLift);
	}
	
	private void intake(){
		if(joystickManager.isIntakeToggled()){
			if(joystickManager.isInverseIntakeToggled()){
				intake_motor.set(-0.5);
			}else{
				intake_motor.set(0.5);
			} 
		}else{
			intake_motor.set(0);
		}
		
	}
	
	private void climb(){
		if(joystickManager.isClimberButtonDepressed()){
			rope_motor.set(1);
		}
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
			if (pilots[(currPilot + 1) % 2].getRawButton(4))// if not-pilot push
															// y
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

		bounds = (int) SmartDashboard.getNumber("DB/Slider 0", bounds);
		bump = SmartDashboard.getNumber("DB/Slider 1", bump);
		factor = SmartDashboard.getNumber("DB/Slider 2", factor);

		if (seesHighGoal) {
			double pixels = visionState.getxPixelOffsetHighGoal();
			if (pixels < bounds * -1 || pixels > bounds) {
				rotationalValue = ((pixels / 160) * factor);// Adjustable
				if (rotationalValue > 0) {
					rotationalValue += bump;// get over hump
				} else {
					rotationalValue -= bump;
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
			// possibly sleep here for a couple ms if needed later
		} else {
			driveRobot(0, 0, 0);
			mode = DriveState.HumanDrive;
		}
		// leftMotor.set(sensorReadingsThread.getRotationValue()); //set to
		// rotationValue
		// rightMotor.set((sensorReadingsThread.getRotationValue()) * -1); //set
		// to inverse of rotationValue

	}

	private void findLift() {
		double xOffset = visionState.getxOffsetLift();// mm
		double psiAngle = Math.toDegrees(visionState.getPsiLift());// rad ->
																	// Degree
		// double thetaAngle = Math.toDegrees(visionState.getThetaLift());//rad
		// -> Degree

		SmartDashboard.putString("DB/String 1", "xOff:" + xOffset);
		SmartDashboard.putString("DB/String 2", "psi:" + psiAngle);

		double angleBump = 0;
		angleBounds = 0;
		angleFactor = 0;

		double transBounds = 0;
		double transBump = 0;
		double transFactor = 0;
		try {
			angleBounds = Double.parseDouble(SmartDashboard.getString("DB/String 5", "10"));
			angleBump = Double.parseDouble(SmartDashboard.getString("DB/String 6", ".1"));
			angleFactor = Double.parseDouble(SmartDashboard.getString("DB/String 7", ".4"));

			transBounds = SmartDashboard.getNumber("DB/Slider 0", 10);
			transBump = SmartDashboard.getNumber("DB/Slider 1", .4);
			transFactor = SmartDashboard.getNumber("DB/Slider 2", .1);
		} catch (Exception e) {
			errored = 1;
		}
		double rotVal = 0;
		double transVal = 0;
		outputRotValue = rotVal;

		if (seesLift) {
			if (psiAngle > angleBounds || psiAngle < -1 * angleBounds) {
				rotVal = psiAngle / 90 * angleFactor;
				if (rotVal > 0)
					rotVal += angleBump;
				else
					rotVal -= angleBump;
				outputRotValue = rotVal;
			}
			if (xOffset > transBounds || transBounds < -100) {
				transVal = xOffset / 600 * transFactor;
				if (transVal > 0)
					transVal += transBump;
				else
					transVal -= transBump;
				outputTransValue = transVal;
			}
			SmartDashboard.putString("DB/String 8", "" + rotVal);
			SmartDashboard.putNumber("DB/Slider 3", transVal);
			if (rotVal == 0 && transVal == 0) {
				mode = DriveState.FindLiftContinue;
			} else {
				driveRobot(transVal, 0, rotVal);
			}
		}
	}

	private void goLift() {
		double zOffset = visionState.getzOffsetLift();// mm
		double transBounds = SmartDashboard.getNumber("DB/Slider 0", 10);
		double transBump = SmartDashboard.getNumber("DB/Slider 1", .4);
		double transFactor = SmartDashboard.getNumber("DB/Slider 2", .1);
		double transVal = 0;
		SmartDashboard.putString("DB/String 9", "zOff:" + zOffset);

		if (zOffset > 700) {
			transVal = zOffset / 1500 * transFactor + transBump;
		} else {
			mode = DriveState.HumanDrive;
		}
		driveRobot(0, transVal, 0);
	}

	private void driveHuman() {
		dPad = SmartDashboard.getBoolean("DB/Button 0", false);

		double x = 0;
		double y = 0;
		double rotation = 0;
		if (activeJoystick().getRawButton(8)) {
			y = .5;
		} else {
			if (!dPad) {
				// moving
				x = activeJoystick().getRawAxis(0);// x of l stick
				y = activeJoystick().getRawAxis(1);// y of l stick
				if (Math.abs(x) <= .1)
					x = 0;
				if (Math.abs(y) <= .1)
					y = 0;
				rotation = activeJoystick().getRawAxis(3) - activeJoystick().getRawAxis(2); // triggers:
																								// right
																								// -
																								// left
																								// to
																								// turn
				if (Math.abs(rotation) <= .1)
					rotation = 0;
			} else {
				if (activeJoystick().getPOV(0) == -1) {
					x = 0;
					y = 0;
				} else {
					x = Math.sin(activeJoystick().getPOV(0) * Math.PI / 180) / 2;
					y = Math.cos(activeJoystick().getPOV(0) * Math.PI / 180) / 2;
				}
			}
		}
		// ^should make 1 when only RT, -1 when only LT
		// may need to make rotation*-1
		// gyroAngle may need to not be 0
		double angle = 0;
		masterDrive.mecanumDrive_Cartesian(x / 2, y / 2, (rotation) / 2, angle);
	}

	private void driveRobot(double x, double y, double rotation) {
		masterDrive.mecanumDrive_Cartesian(x, y, rotation * -1, 0);
	}

	private void shoot() {
		double speed = SmartDashboard.getNumber("DB/Slider 3", 0.0);
		SmartDashboard.putString("DB/String 0", "Speed:" + speed);

		if (activeJoystick().getRawButton(5)) {
			shoot_motor.set(speed);
		}
		if (activeJoystick().getRawButton(6)) {
			shoot_motor.set(0);
		}
	}

	public void turn() {
		if(!isTurning) {
			isTurning = true;
			turningStateMachine.reset();
			turningStateMachine.setInputAngle(Math.toRadians(30));
		}
		
		if(turningStateMachine.isDone()) {
			mode = DriveState.HumanDrive;
		}else{
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
		autoMode =  AutonomousMode.MiddleGear;
	}
	
	@Override
	public void autonomousPeriodic() {
		switch(autoMode) {
		case Selecting:
			break;
		case MiddleGear:
			break;
		case FarGear:
			break;
		case Boiler: 
			break;
		}
	}

	public void valueChanged(ITable source, String key, Object value, boolean isNew) {
	}
	/*
	 * Mappings: A: mode=0 B: mode=1 X: mode=2 Y: Swap Pilots RB: Start Shooter
	 * LB: Stop Shooter SEL: STA: R3: L3:
	 * 
	 * RT: Rot Right LT: Rot Left RS: LS: x-z movement
	 * 
	 * DPAD: x-z movement
	 * 
	 * Strings: 0: Speed(Shoot) 1: PX(High) || xOffset(Lift) 2: RV(High) ||
	 * psiAngle(Lift) 3: Mode 4: seesHighGoal(High)
	 * 
	 * 5:angleBounds (Lift) 6:angleBump(Lift)||encoder.getRaw()
	 * 7:angleFactor(Lift)||encoder.get() 8:angleVal(Lift)||encoder.getRate()
	 * 9:zOffset(Lift)
	 * 
	 * Sliders: 0: Bounds(High)||transBounds(Lift) 1:
	 * Bump(High)||transBumb(Lift) 2: Factor(High)||Factor(Lift) 3:
	 * Speed(Shoot)||transVal(Lift)
	 * 
	 * Buttons: 0: dPad(Drive) 1: 2: 3: seesLift(Lift)
	 * 
	 * buttons: 1 a, 2 b, 3 x, 4 y, 5 lb, 6 rb, 7 back, 8 start, 9 l3, 10 r3
	 * public boolean getRawButton(int button) Axis indexes: 0- LeftX 1 - LeftY
	 * 2 - Left Trigger (0-1) 3 - Right Trigger (0-1) 4 - RightX 5 - RightY
	 * public double getRawAxis(int axis)
	 * 
	 * public int getPOV(int pov) - for d-pad returns degrees from north,
	 * clockwise, -1 if not pressed.
	 * 
	 * SmartDashboard.putString("DB/String 0", "My 21 Char TestString"); String
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
