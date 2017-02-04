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
import edu.wpi.first.wpilibj.GenericHID;
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
   
   private int mode=0;//0=human,1=findHigh,2=findLift
   boolean dPad = false;
   private Joystick[] pilots = new Joystick[2];
   private int currPilot = 0;
   private int rumbleCount;
   
   private static final int F_L_PORT = 7, F_R_PORT = 9, B_L_PORT = 6, B_R_PORT = 8, SHOOT_PORT = 4, ROPE_PORT = 5;
   private SpeedController fore_left_motor, fore_right_motor, back_left_motor, back_right_motor, shoot_motor, rope_motor;

   private int bounds = 3;
   private double bump = 0.3;
   private double factor = .5;
   private boolean seesHighGoal = false;
   
   private VisionState visionState;
    
   private RobotDrive masterDrive;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
   public void robotInit() {
        // Initialize all subsystems
      pilots[0] = new Joystick(0);//0 is joystick import port on driver panel
      pilots[1] = new Joystick(1);
        
      fore_left_motor = new Talon(F_L_PORT);
      fore_right_motor = new Talon(F_R_PORT);
      back_left_motor = new Talon(B_L_PORT);
      back_right_motor = new Talon(B_R_PORT);
   	
      fore_right_motor.setInverted(true);//for whatever reason, right side motors spin wrong way.
      back_right_motor.setInverted(true);//therefore, invert the motors in code.
   	
      masterDrive = new RobotDrive(fore_left_motor, back_left_motor, fore_right_motor, back_right_motor);
   	
      shoot_motor = new Talon(SHOOT_PORT);
      rope_motor = new Talon(ROPE_PORT);
      
      networkTable = NetworkTable.getTable("SmartDashboard");
      networkTable.addTableListener(this);
      
      visionState = new VisionState();
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
      getInput();
      switch(mode)
      {
         case 0:
            swap();
            driveHuman();
            shoot();
            break;
         case 1:
            findHighGoal();
            break;
         case 2:
            findLift();
            break;
      }   
   }

   private void getInput()
   {
      if(pilots[currPilot].getRawButton(1))
      {
         mode = 0;
         SmartDashboard.putString("DB/String 3","Mode:"+mode);
      
      }
      if(pilots[currPilot].getRawButton(2))
      {
         mode = 1;
         SmartDashboard.putString("DB/String 3","Mode:"+mode);
      }
      if(pilots[currPilot].getRawButton(3))
      {
         mode = 2;
         SmartDashboard.putString("DB/String 3","Mode:"+mode);
      }
      
      seesHighGoal = visionState.seesHighGoal();
      SmartDashboard.putString("DB/String 4", "TP:"+seesHighGoal);
   }
   
   private void swap(){
      if(rumbleCount==0)
      {
         pilots[(currPilot+1) % 2].setRumble(GenericHID.RumbleType.kLeftRumble,0);
         pilots[(currPilot+1) % 2].setRumble(GenericHID.RumbleType.kRightRumble,0);	
      }
      if(rumbleCount<0)
      {
         if(pilots[(currPilot+1) % 2].getRawButton(4))//if not-pilot push y
         {
            pilots[currPilot].setRumble(GenericHID.RumbleType.kLeftRumble,1);
            pilots[currPilot].setRumble(GenericHID.RumbleType.kRightRumble,1);
            currPilot = (currPilot+1) % 2;//switch pilot 
            rumbleCount = 10;
         }
      }
      else
      {
         rumbleCount--;
      }
   }
   
   private void findHighGoal()
   {
      double rotationalValue;
         
      bounds = (int)SmartDashboard.getNumber("DB/Slider 0",bounds);
      bump = SmartDashboard.getNumber("DB/Slider 1",bump);
      factor = SmartDashboard.getNumber("DB/Slider 2",factor);
         
      if (seesHighGoal) {
         double pixels = visionState.getxOffsetHighGoal();
         if (pixels < bounds*-1 || pixels > bounds) {
            rotationalValue = ((pixels / 160) * factor);//Adjustable
            if(rotationalValue>0)
            {
               rotationalValue+=bump;//get over hump
            }
            else
            {
               rotationalValue-=bump;
            }
               
            driveRobot(0,0,rotationalValue);
               	
            SmartDashboard.putString("DB/String 2", "RV="+rotationalValue);
            SmartDashboard.putString("DB/String 1", "PX="+visionState.getxOffsetHighGoal());
         }
         else {
            driveRobot(0,0,0);
            mode = 0;
            	//perform high goal
            	//return control to teleop
         }
         	//possibly sleep here for a couple ms if needed later
      }
      else
      {
         driveRobot(0,0,0);
         mode=0;
      }
       	//leftMotor.set(sensorReadingsThread.getRotationValue()); //set to rotationValue
       	//rightMotor.set((sensorReadingsThread.getRotationValue()) * -1); //set to inverse of rotationValue
   
   }
   
   private void findLift()
   {
      boolean seesLift = visionState.seesLift();
      double xOffset = visionState.getxOffsetLift();//mm
      double zOffset = visionState.getzOffsetLift();//mm
      double psiAngle = Math.toDegrees(visionState.getPsiLift());//rad -> Degree
      //double thetaAngle = Math.toDegrees(visionState.getThetaLift());//rad -> Degree
      
      SmartDashboard.putString("DB/String 1", "xOff:"+xOffset);
      SmartDashboard.putString("DB/String 2", "psi:"+psiAngle);
      SmartDashboard.putString("DB/String 9", "zOff:"+zOffset);
      
      
      double angleBounds = Double.parseDouble(SmartDashboard.getString("DB/String 5","10"));
      double angleFactor = Double.parseDouble(SmartDashboard.getString("DB/String 6",".3"));
      
      double transBounds = Double.parseDouble(SmartDashboard.getString("DB/String 7","100"));
      double transFactor = Double.parseDouble(SmartDashboard.getString("DB/String 8",".5"));
      
      double rotVal = 0;
      double transVal = 0;
      
      if(seesLift)
      {
         if(psiAngle>angleBounds||psiAngle<-1*angleBounds)
         {
        	 rotVal = psiAngle/90*angleFactor;
         }
         if(xOffset>transBounds||transBounds<-100)
         {
        	 transVal = xOffset/300*transFactor;
         }
         if(rotVal == 0 && transVal == 0)
         {
        	 if(zOffset>100)
        	 {
        		 transVal = zOffset/500*transFactor;
        		 driveRobot(0,transVal,0);
        	 }
        	 else
        	 {
        		 mode = 0;
        	 }
         }
         else
         {
        	 driveRobot(transVal,0,rotVal);
         }
      }
   }
   
   private void driveHuman(){
      dPad = SmartDashboard.getBoolean("DB/Button 0", false);
   
      double x = 0;
      double y = 0;
      double rotation = 0;
      if(!dPad)
      {
      	//moving
         x = pilots[currPilot].getRawAxis(0);//x of l stick
         y = pilots[currPilot].getRawAxis(1);//y of l stick
         rotation = pilots[currPilot].getRawAxis(3) - pilots[currPilot].getRawAxis(2); //triggers: right - left to turn
      }
      else
      {
         x = Math.cos(pilots[currPilot].getPOV(0)*Math.PI/180);
         y = Math.sin(pilots[currPilot].getPOV(0)*Math.PI/180);
      }
         // ^should make 1 when only RT, -1 when only LT
    	//may need to make rotation*-1
        //gyroAngle may need to not be 0
      masterDrive.mecanumDrive_Cartesian(x,y,rotation,0);
   }
    
   private void driveRobot(double x,double y,double rotation){
      masterDrive.mecanumDrive_Cartesian(x,y,rotation,0);
   }
   
   private void shoot(){
      double speed = SmartDashboard.getNumber("DB/Slider 3", 0.0);
      SmartDashboard.putString("DB/String 0", "Speed:"+speed);
      
      if(pilots[currPilot].getRawButton(5))
      {
         shoot_motor.set(speed);
      }
      if(pilots[currPilot].getRawButton(6))
      {
         shoot_motor.set(0);
      }
   }
    /**
     * This function is called periodically during test mode
     */
   public void testPeriodic() {
      
   }

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
   private void log() {
        //wrist.log();
   }
   public void valueChanged(ITable source, String key, Object value, boolean isNew) {
   }
   /*
   Mappings:
   A: mode=0
   B: mode=1
   X: mode=2
   Y: Swap Pilots
   RB: Start Shooter
   LB: Stop Shooter
   SEL:
   STA:
   R3:
   L3:
   
   RT: Rot Right
   LT: Rot Left
   RS: 
   LS: x-z movement
   
   DPAD: x-z movement
   
   Strings:
   0: Speed(Shoot)
   1: PX(High) || xOffset(Lift)
   2: RV(High) || psiAngle(Lift)
   3: Mode
   4: seesHighGoal(High)
   
   5:angleBounds (Lift)
   6:angleFactor(Lift)
   7:transBounds(Lift)
   8:transFactor(Lift)
   9:zOffset(Lift)
   
   Sliders:
   0: Bounds(High)
   1: Bump(High)
   2: Factor(High)
   3: Speed(Shoot)
   
   Buttons:
   0: dPad(Drive)
   1:
   2:
   3:
   
   buttons: 1 a, 2 b, 3 x, 4 y, 5 lb, 6 rb, 7 back, 8 start, 9 l3, 10 r3
   public boolean getRawButton(int button)
   Axis indexes:
0- LeftX
1 - LeftY
2 - Left Trigger (0-1)
3 - Right Trigger (0-1)
4 - RightX
5 - RightY
   public double getRawAxis(int axis)

   public int getPOV(int pov) - for d-pad
      returns degrees from north, clockwise, -1 if not pressed.
   
    SmartDashboard.putString("DB/String 0", "My 21 Char TestString");
    String dashData = SmartDashboard.getString("DB/String 0", "myDefaultData");
   
    SmartDashboard.putBoolean("DB/Button 0", true);
    (default value of false): boolean buttonValue = SmartDashboard.getBoolean("DB/Button 0", false);

    SmartDashboard.putNumber("DB/Slider 0", 2.58);
    (default value of 0.0): double dashData = SmartDashboard.getNumber("DB/Slider 0", 0.0);

      
   */
}
