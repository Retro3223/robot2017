package org.usfirst.frc.team3223.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class SensorManager {
	private AHRS ahrs;
	long currentTime; // ms
	long previousTime;// ms
	double XAccel;
	double YAccel;
	double ZAccel;
	//double currentXAccel;
	//double currentYAccel;
	//double currentZAccel;
	
	public SensorManager() {
		ahrs = new AHRS(SPI.Port.kMXP);
	}
	
	public void tick() {
		previousTime = currentTime;
		currentTime = System.currentTimeMillis();
		XAccel = ahrs.getWorldLinearAccelX();
		YAccel = ahrs.getWorldLinearAccelY();
		ZAccel = ahrs.getWorldLinearAccelZ();
		
	}
	
	/**
	 * @return current heading (degrees)
	 */
	public double getAngle() {
		return ahrs.getAngle();
	}
	
	/**
	 * @return current heading (radians)
	 */
	public double headingRad() {
		return Math.toRadians(ahrs.getAngle());
	}
	
	/**
	 * @return change in heading from.. the last tick? (degrees)
	 */
	public double getDeltaAngle() {
		return ahrs.getRate();
	}
	
	/** 
	 * @return angular velocity (yaw) (rad/s)
	 */
	public double getAngularVelocity() {
		return Math.toRadians(getDeltaAngle())/((currentTime-previousTime)/1000.000);
	}
	
	/*public double getXJerk(){
		return (currentXAccel-previousXAccel)/(currentTime-previousTime);
	}
	
	public double getYJerk(){
		return (currentYAccel-previousYAccel)/(currentTime-previousTime);
	}*/
	
	public double getXAccel(){
		return this.XAccel;
	}
	
	public double getYAccel(){
		return this.YAccel;
	}
	
	public double getZAccel(){
		return this.ZAccel;
	}
}