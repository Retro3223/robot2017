package org.usfirst.frc.team3223.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickManager {
	private boolean leftDPADWasPressed;
	private boolean leftDPADIsToggled;
	private boolean rightDPADWasPressed;
	private boolean rightDPADIsToggled;
	private Supplier<Joystick> getActiveJoystick;
	
	public boolean isLeftDPAD(){
		return activeJoystick().getPOV(0) == 270;
	}
	
	public boolean isRightDPAD(){
		return activeJoystick().getPOV(0) == 90;
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
	
	public boolean isClimberButtonDepressed(){
		return activeJoystick().getRawButton(10);
	}
	
	public void tick(){
		if(!leftDPADWasPressed && isLeftDPAD()){
			leftDPADIsToggled = !leftDPADIsToggled;
		}
		if(!rightDPADWasPressed && isRightDPAD()){
			rightDPADIsToggled = !rightDPADIsToggled;
		}
		leftDPADWasPressed = isLeftDPAD();
		rightDPADWasPressed = isRightDPAD();
	}
	
	public JoystickManager(Supplier<Joystick> getActiveJoystick){
		this.getActiveJoystick = getActiveJoystick;
		leftDPADWasPressed = false;
		rightDPADWasPressed = false;
	}
}
