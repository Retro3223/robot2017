package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;

public class ControlledDrive {
	VelocityRegulator frontLeft, backLeft, frontRight, backRight;
	VelocityRegulator[] regulators;
	private RobotDrive robotDrive;
	
	public ControlledDrive(SpeedController[] speedControllers, Encoder[] encoders) {
		regulators = new VelocityRegulator[4];
		for(int i = 0; i < 4; i++) {
			encoders[i].setPIDSourceType(PIDSourceType.kRate);
			regulators[i] = new VelocityRegulator(speedControllers[i], encoders[i]);
		}
		frontLeft = regulators[0];
		backLeft = regulators[1];
		frontRight = regulators[2];
		backRight = regulators[3];
		
		robotDrive = new RobotDrive(
				frontLeft.asSpeedControllerForMechanumDrive(), 
				backLeft.asSpeedControllerForMechanumDrive(), 
				frontRight.asSpeedControllerForMechanumDrive(), 
				backRight.asSpeedControllerForMechanumDrive());
	   	
		
	}
	
	public void enable()
	{
		for(int i =0 ; i < 4; i++){
			regulators[i].enable();
		}
	}
	
	public void disable()
	{
		for(int i =0 ; i < 4; i++){
			regulators[i].disable();
		}
	}
	
	public void drive(double x, double y, double rotation, double gyroAngle) {
		robotDrive.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
	}
}
