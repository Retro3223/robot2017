package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

public class VelocityRegulator {
	Encoder encoder;
	SpeedController sController;
	PIDController pController;
	
	
	
	public VelocityRegulator(SpeedController sController, Encoder encoder){
		this.sController = sController;
		this.encoder = encoder;
		this.pController = new PIDController(0.5, 0, 0, 0, encoder, sController);
		pController.setInputRange(-2, 2);
		pController.setOutputRange(-1, 1);
		pController.setAbsoluteTolerance(.02);
	}
	
	public void enable(){
		pController.enable();
	}
	
	public void disable(){
		pController.disable();
	}
	
	public void setVelocity(double setpoint){
		pController.setSetpoint(setpoint);
	}
}
