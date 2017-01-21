/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot implements ITableListener{
    Command autonomousCommand;
    NetworkTable networkTable;
    private double motorSpeed = 0;
    private static final int F_L_PORT = 0, F_R_PORT = 1, B_L_PORT = 2, B_R_PORT = 3;
    private Joystick joystick;
    /*buttons: 1 a, 2 b, 3 x, 4 y, 5 lb, 6 rb, 7 back, 8 start, 9 l3, 10 r3
    Axis indexes:
0- LeftX
1 - LeftY
2 - Left Trigger (0-1)
3 - Right Trigger (0-1)
4 - RightX
5 - RightY

public int getPOV(int pov) - for d-pad
returns degrees from north, clockwise, -1 if not pressed.
    */
    private SpeedController fore_left_motor, fore_right_motor, back_left_motor, back_right_motor;
    private RobotDrive masterDrive;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        // Initialize all subsystems
        joystick = new Joystick(0);//0 is joystick import port on driver panel
        
        fore_left_motor = new Talon(F_L_PORT);
		fore_right_motor = new Talon(F_R_PORT);
		back_left_motor = new Talon(B_L_PORT);
		back_right_motor = new Talon(B_R_PORT);
		masterDrive = new RobotDrive(fore_left_motor, back_left_motor, fore_right_motor, back_right_motor);
		
		networkTable = NetworkTable.getTable("SmartDashboard");
		networkTable.addTableListener(this);
        // instantiate the command used for the autonomous perio

        // Show what command your subsystem is running on the SmartDashboard
        //SmartDashboard.putData(drivetrain);
        
    }

  
    public void teleopInit() {
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	double x = joystick.getRawAxis(0);//x of l stick
        double y = joystick.getRawAxis(1);//y of l stick
        double rotation = joystick.getRawAxis(3) - joystick.getRawAxis(2); //triggers: right - left to turn
        // ^should make 1 when only RT, -1 when only LT
    	//may need to make rotation*-1
        //gyroAngle may need to not be 0
        masterDrive.mecanumDrive_Cartesian(l,r,rotation,0);//x,y,rotation,gyroAngle)
    	/*
    	fore_left_motor.set(motorSpeed);
    	*/
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	
    }
    @Override
    public void valueChanged(ITable table, String name, Object value, boolean isNew) {
    	if(name.equals("motorSpeed"))
    		motorSpeed= (double) value;
    }

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
    private void log() {
        //wrist.log();
        
    }
}
