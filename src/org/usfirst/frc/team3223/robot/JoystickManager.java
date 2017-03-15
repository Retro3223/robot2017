package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class JoystickManager {
	NetworkTable networkTable;
	private Joystick[] pilots = new Joystick[2];
	private int currPilot = 0;
	private int rumbleCount;
	private boolean climberButtonWasPressed;
	private boolean climberButtonToggled;
	private boolean reverseClimberButtonWasPressed;
	private boolean reverseClimberButtonToggled;
	
	public JoystickManager(){
		pilots[0] = new Joystick(0);// 0 is joystick import port on driver panel
		pilots[1] = new Joystick(1);
		networkTable = NetworkTable.getTable("SmartDashboard");
	}
	
	public Joystick activeJoystick() {
		return pilots[currPilot];
	}

	private void publishJoystickState() {
		networkTable.putBoolean("Left DPAD Pressed", isLeftDPAD());
		networkTable.putBoolean("Right DPAD Pressed", isRightDPAD());
		networkTable.putBoolean("Up DPAD Pressed", isUpDPAD());
		networkTable.putBoolean("Down DPAD Pressed", isDownDPAD());
		networkTable.putBoolean("A Pressed", activeJoystick().getRawButton(1));
		networkTable.putBoolean("B Pressed", activeJoystick().getRawButton(2));
		networkTable.putBoolean("X Pressed", activeJoystick().getRawButton(3));
		networkTable.putBoolean("Y Pressed", activeJoystick().getRawButton(4));
		networkTable.putBoolean("Right Bumper Pressed", activeJoystick().getRawButton(6));
		networkTable.putBoolean("Left Bumper Pressed", activeJoystick().getRawButton(5));
		networkTable.putBoolean("Left Stick Pressed", activeJoystick().getRawButton(9));
		networkTable.putBoolean("Right Stick Pressed", activeJoystick().getRawButton(10));
		networkTable.putBoolean("Guide Pressed", activeJoystick().getRawButton(8));
		networkTable.putBoolean("Back Pressed", activeJoystick().getRawButton(7));
	}
	
	private void publishTeleopJoystickControls() {
		networkTable.putString("Left DPAD Text", "Slurp fuel");
		networkTable.putString("Right DPAD Text", "Eject fuel");
		networkTable.putString("Up DPAD Text", "Shooter on");
		networkTable.putString("Down DPAD Text", "Shooter off");
		networkTable.putString("A Text", "Be human driven");
		networkTable.putString("B Text", "Find high goal");
		networkTable.putString("X Text", "Find lift");
		networkTable.putString("Y Text", "Swap pilots");
		networkTable.putString("Right Bumper Text", "Open Trapdoor");
		networkTable.putString("Left Bumper Text", "Contemplate life");
		networkTable.putString("Left Stick Text", "Drive");
		networkTable.putString("Right Stick Text", "Eat sandwich");
		networkTable.putString("Guide Text", "Climb (reverse direction)");
		networkTable.putString("Back Text", "Climb");
	}
	
	public boolean isLeftDPAD(){
		return activeJoystick().getPOV(0) == 270;
	}
	
	public boolean isRightDPAD(){
		return activeJoystick().getPOV(0) == 90;
	}

	public boolean isUpDPAD() {
		return activeJoystick().getPOV(0) == 0;
	}
	
	public boolean isDownDPAD() {
		return activeJoystick().getPOV(0) == 180;
	}
	
	public boolean isR3(){
		return activeJoystick().getRawButton(10);
	}
	
	public boolean isClimberButtonDepressed(){
		return activeJoystick().getRawButton(7);
	}
	public boolean isClimberButtonToggled() {
		return climberButtonToggled;
	}

	public boolean isReverseClimberButtonDepressed(){
		return activeJoystick().getRawButton(8);
	}
	
	public boolean isReverseClimberButtonToggled() {
		return reverseClimberButtonToggled;
	}

    public boolean swapJoystickButtonPressed() {
        return activeJoystick().getRawButton(4);
    }

    public boolean humanDriveButtonPressed() {
		return activeJoystick().getRawButton(1);
    }

    public boolean findHighGoalButtonPressed() {
		return activeJoystick().getRawButton(2);
    }

    public boolean findLiftButtonPressed() {
		return activeJoystick().getRawButton(3);
    }
    
    public boolean openTrapdoorButtonPressed() {
        return activeJoystick().getRawButton(6);
    }

    public boolean shootButtonPressed() {
        return isUpDPAD();
    }

    public boolean noShootButtonPressed() {
        return isDownDPAD();
    }

	public void swapActiveJoystick() {
		if (rumbleCount >= 0) {
			pilots[(currPilot + 1) % 2].setRumble(GenericHID.RumbleType.kLeftRumble, 0);
			pilots[(currPilot + 1) % 2].setRumble(GenericHID.RumbleType.kRightRumble, 0);
		}
		if (rumbleCount < 0) {
			if (pilots[(currPilot + 1) % 2].getRawButton(4))// if not-pilot push 'y'
			{
				pilots[currPilot].setRumble(GenericHID.RumbleType.kLeftRumble, 1);
				pilots[currPilot].setRumble(GenericHID.RumbleType.kRightRumble, 1);
				currPilot = (currPilot + 1) % 2;// switch pilot
				rumbleCount = 10;
			}
		} else {
			rumbleCount--;
		}
	}
	
	public void tick() {
		if(!climberButtonWasPressed && isClimberButtonDepressed()) {
			climberButtonToggled = !climberButtonToggled;
		}
		if(!reverseClimberButtonWasPressed && isReverseClimberButtonDepressed()) {
			reverseClimberButtonToggled = !reverseClimberButtonToggled;
		}
			
		climberButtonWasPressed = isClimberButtonDepressed();
		reverseClimberButtonWasPressed = isReverseClimberButtonDepressed();

		this.publishJoystickState();
		this.publishTeleopJoystickControls();
	}

}
