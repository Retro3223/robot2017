package org.usfirst.frc.team3223.robot;

import org.usfirst.frc.team3223.enums.TranslationalState;
import org.usfirst.frc.team3223.enums.TurningState;

public class TranslationalStateMachine {
	RotationalProfiler profiler;
	TranslationalState state;
	double inputDistance;
	double velocity;
	long currentTime; //ms
	long startTime;//ms
	long timeDelta;
	Robot robot;//ms
	double voltage;
	
	
	public TranslationalStateMachine(Robot robot){
		this.profiler = new RotationalProfiler(20, 10);
		state = TranslationalState.Start;
		this.robot = robot;
	}
	
	public void setInputDistance(double distance){
		this.inputDistance = distance;
	}
	
	public void reset(){
		state = TranslationalState.Start;
	}
	
	public void run(){
		currentTime = System.currentTimeMillis();
		switch (state){
		case Start:
			state = TranslationalState.Calculate;
			break;
		case Calculate:
			profiler.calculate(inputDistance);
			state = TranslationalState.Drive;
			startTime = currentTime;
			break;
		case Drive:
			timeDelta = currentTime-startTime;
			velocity = profiler.getVelocity(timeDelta);
			voltage = velocity*.05;
			robot.driveRobot(0, voltage, 0);
			if(profiler.isDone(timeDelta)){
				state = TranslationalState.End;
			}
			break;
		case End:
			robot.driveRobot(0, 0, 0);
			break;
		}
	}
	
	public boolean isDone(){
		return state == TranslationalState.End;
}
	
	
}
