package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.SpeedController;

public class BangBang {
	Counter counter;
	SpeedController speedController;
	double desiredRPM;
	
	public BangBang(Counter counter, SpeedController speedController) {
		this.counter = counter;
		this.speedController = speedController;
	}
	
	public void setSetpoint(double desiredRPM) {
		this.desiredRPM = desiredRPM;
	}
	
	private double measuredRPM() {
		return counter.getRate() * 60.0;
	}
	
	public void run() {
		if(this.measuredRPM() > desiredRPM) {
			speedController.set(0);
		}else{
			speedController.set(1);
		}
	}
}
