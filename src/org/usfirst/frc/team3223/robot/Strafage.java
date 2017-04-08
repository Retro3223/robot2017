package org.usfirst.frc.team3223.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.usfirst.frc.team3223.enums.StrafeState;
import org.usfirst.frc.team3223.enums.TranslationalState;

import edu.wpi.first.wpilibj.Encoder;

public class Strafage {
	
	private Encoder encoder;
	RotationalProfiler profiler;
	StrafeState profilerState;
	Supplier<Integer> getState;
	Consumer<Integer> setState;
	private double distanceToTravel;
	private double distanceOffset = 0;
	double okThresholdDistance = 20;
	double decelerateThresholdDistance = 50; // todo: calibrate
	private int strafyState = 0; // 0 -> init, 1 -> strafe left, 2 -> strafe right, 3 -> done
	long currentTime;
	long startTime;
	
	public Strafage(Encoder encoder, Supplier<Integer> getState, Consumer<Integer> setState) {
		this.encoder = encoder;
		this.encoder.setDistancePerPulse(1.4); // todo: calibrate
		this.getState = getState;
		this.setState = setState;
		
		profiler = new RotationalProfiler(40, 40);
	}
	
	public void setupLogging(RecorderContext context) {

		context.add("encoder distance", () -> encoder.getDistance());
		context.add("distance to travel", () -> distanceToTravel);
		context.add("strafe state", () -> strafyState);
	}
	
	public void reset() {
		strafyState = 0;
		profilerState = StrafeState.Init;
		encoder.reset();
	}
	
	public void setDistance(double distance_mm) {
		this.distanceToTravel = distance_mm;
		if((strafyState == 1 || strafyState == 2) && Math.abs(distance_mm) > okThresholdDistance + distanceOffset) {
			reinit();
		}
		
	}
	
	private void reinit() {
		this.encoder.reset();
		if(distanceToTravel > okThresholdDistance + distanceOffset) {
			strafyState = 1;
		}else if(distanceToTravel < -okThresholdDistance + distanceOffset){
			strafyState = 2;
		}
	}
	public void strafe(Robot robot, int strafeState, int endState) {
		
		if(strafeState == this.getState.get() && strafyState == 0){
			reinit();
		}else if(strafyState == 1) {
			// strafe right
			double distanceTraveled = this.encoder.getDistance();
			robot.driveRobot(0.4, 0, 0);
			if(distanceToTravel - distanceTraveled < decelerateThresholdDistance) {
				strafyState = 3;
			}
		}else if(strafyState == 2) {
			// strafe left
			double distanceTraveled = this.encoder.getDistance();
			robot.driveRobot(-0.4, 0, 0);
			if(distanceToTravel - distanceTraveled > -decelerateThresholdDistance) {
				strafyState = 3;
			}
		}else if(strafyState == 3) {
			robot.driveRobot(0, 0, 0);
			this.setState.accept(endState);
		}
	}
	
	public void strafe3(Robot robot, int strafeState, int endState) {
		currentTime = System.currentTimeMillis();
		if(profilerState == StrafeState.Init && strafeState == this.getState.get()) {
			profiler.calculate(distanceToTravel);
			startTime = currentTime;
			profilerState = StrafeState.Drive;
		}else if(profilerState == StrafeState.Drive) {
			long timeDelta = currentTime-startTime;
			double velocity = profiler.getVelocity(timeDelta);
			double voltage = velocity*.025;
			robot.driveRobot(-voltage, 0, 0);
			if(profiler.isDone(timeDelta)){
				profilerState = StrafeState.End;
			}
		}else if(profilerState == StrafeState.End) {
			robot.driveRobot(0, 0, 0);
			this.setState.accept(endState);
			profilerState = StrafeState.Init;
		}
	}
}
