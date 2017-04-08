package org.usfirst.frc.team3223.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.usfirst.frc.team3223.enums.StrafeProfileState;
import org.usfirst.frc.team3223.enums.StrafeState;
import org.usfirst.frc.team3223.enums.TranslationalState;

import edu.wpi.first.wpilibj.Encoder;

public class Strafage {
	
	private Encoder encoder;
	RotationalProfiler profiler;
	StrafeProfileState profilerState;
	Supplier<Integer> getState;
	Consumer<Integer> setState;
	private double distanceToTravel;
	private double sumDistanceTraveled;
	private double distanceOffset = 0;
	double okThresholdDistanceP = 20;

	double okThresholdDistanceN = -20;
	double decelerateThresholdDistanceP = 50; // todo: calibrate

	double decelerateThresholdDistanceN = -50; // todo: calibrate
	private StrafeState strafyState = StrafeState.Init; 
	long currentTime;
	long startTime;
	
	public Strafage(Encoder encoder, Supplier<Integer> getState, Consumer<Integer> setState) {
		this.encoder = encoder;
		this.encoder.setDistancePerPulse(1.4);
		this.getState = getState;
		this.setState = setState;
		
		profiler = new RotationalProfiler(40, 40);
	}
	
	public void setupLogging(RecorderContext context) {

		context.add("encoder distance", () -> encoder.getDistance());
		context.add("distance to travel", () -> distanceToTravel);
		context.add("distance traveled", () -> sumDistanceTraveled + encoder.getDistance());
		context.add("strafe state", () -> strafyState.toString());
	}
	
	public void reset() {
		sumDistanceTraveled = 0;
		strafyState = StrafeState.Init;
		profilerState = StrafeProfileState.Init;
		encoder.reset();
	}
	
	public void setDistance(double distance_mm) {
		this.distanceToTravel = distance_mm;
		if((strafyState == StrafeState.Right || strafyState == StrafeState.Left) && 
				(distance_mm > okThresholdDistanceP || distance_mm < okThresholdDistanceN)) {
			sumDistanceTraveled += this.encoder.getDistance();
			reinit();
		}
		
	}
	
	private void reinit() {
		this.encoder.reset();
		if(distanceToTravel > okThresholdDistanceP) {
			strafyState = StrafeState.Right;
		}else if(distanceToTravel < okThresholdDistanceN){
			strafyState = StrafeState.Left;
		}
	}
	
	public void strafe(Robot robot, int beStrafingState, int endState) {
		
		if(beStrafingState == this.getState.get() && strafyState == StrafeState.Init){
			reinit();
		}else if(strafyState == StrafeState.Right) {
			// strafe right
			double distanceTraveled = this.encoder.getDistance();
			robot.driveRobot(0.4, 0, 0);
			if(distanceToTravel - distanceTraveled < decelerateThresholdDistanceP) {
				strafyState = StrafeState.Done;
			}
		}else if(strafyState == StrafeState.Left) {
			// strafe left
			double distanceTraveled = this.encoder.getDistance();
			robot.driveRobot(-0.4, 0, 0);
			if(distanceToTravel - distanceTraveled > decelerateThresholdDistanceN) {
				strafyState = StrafeState.Done;
			}
		}else if(strafyState == strafyState) {
			robot.driveRobot(0, 0, 0);
			this.setState.accept(endState);
		}
	}
	
	public void strafe3(Robot robot, int strafeState, int endState) {
		currentTime = System.currentTimeMillis();
		if(profilerState == StrafeProfileState.Init && strafeState == this.getState.get()) {
			profiler.calculate(distanceToTravel);
			startTime = currentTime;
			profilerState = StrafeProfileState.Drive;
		}else if(profilerState == StrafeProfileState.Drive) {
			long timeDelta = currentTime-startTime;
			double velocity = profiler.getVelocity(timeDelta);
			double voltage = velocity*.025;
			robot.driveRobot(-voltage, 0, 0);
			if(profiler.isDone(timeDelta)){
				profilerState = StrafeProfileState.End;
			}
		}else if(profilerState == StrafeProfileState.End) {
			robot.driveRobot(0, 0, 0);
			this.setState.accept(endState);
			profilerState = StrafeProfileState.Init;
		}
	}
}
