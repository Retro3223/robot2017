package org.usfirst.frc.team3223.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickManager {
	private boolean leftDPADWasPressed;
	private boolean leftDPADIsToggled;
	private boolean rightDPADWasPressed;
	private boolean rightDPADIsToggled;
	private boolean upDPADWasPressed;
	private boolean upDPADIsToggled;
	private boolean R3WasPressed;
	private boolean R3IsToggled;
	
	private Supplier<Joystick> getActiveJoystick;
	
	public boolean isLeftDPAD(){
		return activeJoystick().getPOV(0) == 270;
	}
	
	public boolean isRightDPAD(){
		return activeJoystick().getPOV(0) == 90;
	}
	
	public boolean isUpDPAD(){
		return activeJoystick().getPOV(0) == 0;
	}
	
	public boolean isR3(){
		return activeJoystick().getRawButton(10);
	}
	
	private Joystick activeJoystick(){
		return this.getActiveJoystick.get();
	}
	
	public boolean isIntakeToggled(){
		return leftDPADIsToggled;
	}
	
	public boolean isInverseIntakeToggled(){
		return rightDPADIsToggled;
	}
	
	public boolean isInvertToggled(){
		return R3IsToggled;
	}
	
	public boolean isShooterToggled(){
		return upDPADIsToggled;
	}
	
	public boolean isClimberButtonDepressed(){
		return activeJoystick().getRawButton(7);
	}

	public boolean isReverseClimberButtonDepressed(){
		return activeJoystick().getRawButton(8);
	}
	
	public void tick(){
		if(!leftDPADWasPressed && isLeftDPAD()){
			leftDPADIsToggled = !leftDPADIsToggled;
		}
		if(!rightDPADWasPressed && isRightDPAD()){
			rightDPADIsToggled = !rightDPADIsToggled;
		}
		if(!upDPADWasPressed && isUpDPAD()){
			upDPADIsToggled = !upDPADIsToggled;
		}
		if(!R3WasPressed && isR3()){
			R3IsToggled = !R3IsToggled;
		}
			
		leftDPADWasPressed = isLeftDPAD();
		rightDPADWasPressed = isRightDPAD();
		upDPADWasPressed = isUpDPAD();
		R3WasPressed = isR3();
	}
	
	public JoystickManager(Supplier<Joystick> getActiveJoystick){
		this.getActiveJoystick = getActiveJoystick;
		leftDPADWasPressed = false;
		rightDPADWasPressed = false;
	}

	public boolean isUpDPAD() {
		return activeJoystick().getPOV(0) == 0;
	}
	
	public boolean isDownDPAD() {
		return activeJoystick().getPOV(0) == 180;
	}
}
