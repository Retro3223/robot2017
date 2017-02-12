package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

public class VelocityRegulator {
	Encoder encoder;
	SpeedController sController;
	PIDController pController;
	double maxSpeed = 3500;
	Talon t;
	
	double speedPercent;
	boolean isInverted;
	
	public VelocityRegulator(SpeedController sController, Encoder encoder){
		this.sController = sController;
		this.encoder = encoder;
		this.pController = new PIDController(0.005, 0, 0, 0, encoder, sController);
		pController.setInputRange(-maxSpeed, maxSpeed);
		pController.setOutputRange(-1, 1);
		pController.setAbsoluteTolerance(.02);
	}
	
	public void enable(){
		pController.enable();
	}
	
	public void disable(){
		pController.disable();
	}
	
	public double getVelocity() {
		return speedPercent * maxSpeed;
	}
	
	public void setVelocity(double speedPercent){
		this.speedPercent = speedPercent;
		pController.setSetpoint(speedPercent * maxSpeed);
	}
	
	/**
	 * pretend to be a speed controller, so we can reuse RobotDrive's mechanum code
	 * @return
	 */
	public SpeedController asSpeedControllerForMechanumDrive() {
		return new SpeedController() {

			@Override
			public void pidWrite(double output) {
				throw new UnsupportedOperationException("not implemented");
				
			}

			@Override
			public double get() {
				return VelocityRegulator.this.speedPercent; 
			}

			@Override
			public void set(double speedPercent) {
				VelocityRegulator.this.setVelocity(speedPercent);
				
			}

			@Override
			public void setInverted(boolean isInverted) {
				throw new UnsupportedOperationException("not implemented");
				
			}

			@Override
			public boolean getInverted() {
				return false;
			}

			@Override
			public void disable() {
			}

			@Override
			public void stopMotor() {
				
			}
			
		};
	}
}
