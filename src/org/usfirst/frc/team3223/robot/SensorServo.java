package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;

public class SensorServo {
	private DigitalInput highGoalPosition;
	private DigitalInput gearPosition;
	private Servo servo;
	boolean goToHighGoal;
	boolean goToGear;
	
	public boolean inHighGoalPosition(){
		return highGoalPosition.get();
	}
	
	public boolean inGearPosition(){
		return gearPosition.get();
	}
	
	public void goToHighGoalPosition(){
		goToHighGoal = true;
	}
	
	public void goToGearPosition(){
		goToGear = true;
	}
	
	public void tick(){
		if(goToHighGoal){
			if(inHighGoalPosition()){
				servo.set(0);
				goToHighGoal = false;
			}else{
				servo.set(0.5);
			}
		}else if(goToGear){
			if(inGearPosition()){
				servo.set(0);
				goToGear = false;
			}else{
				servo.set(-0.5);
			}
		}else{
			servo.set(0);
		}
	}
	
	public SensorServo(DigitalInput highGoalPosition, DigitalInput gearPosition, Servo servo){
		this.highGoalPosition = highGoalPosition;
		this.gearPosition = gearPosition;
		this.servo = servo;
		this.goToGear = false;
		this.goToHighGoal = false;
	}
}
