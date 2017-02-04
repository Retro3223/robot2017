package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class VisionState implements ITableListener {
   private boolean seesHighGoal;
   private double xOffsetHighGoal;
   
   private boolean seesLift;
   private double xOffsetLift; //in mm
   private double zOffsetLift; //in mm
   private double thetaLift; //in rads - angle between robot dir and lift location
   private double psiLift; //in rads - angle between robot dir and wall (perbendicular)
	
   public VisionState() {
      NetworkTable networkTable = NetworkTable.getTable("SmartDashboard");
      networkTable.addTableListener(this);
   }

   @Override
   public void valueChanged(ITable source, String key, Object value, boolean isNew) {
      if(key.equals("seesHighGoal")) {
         seesHighGoal = (boolean) value;
      }
      if(key.equals("xOffsetHighGoal")) {
         xOffsetHighGoal = (double) value;
      }
      if(key.equals("seesLift")) {
         seesLift = (boolean) value;
      }
      if(key.equals("xOffsetLift")) {
         xOffsetLift = (double) value;
      }
      if(key.equals("zOffsetLift")) {
         zOffsetLift = (double) value;
      }
      if(key.equals("thetaLift")) {
         thetaLift = (double) value;
      }
      if(key.equals("psiLift")) {
         psiLift = (double) value;
      }
   
   }

   public boolean seesHighGoal() {
      return seesHighGoal;
   }
	
   public double getxOffsetHighGoal() {
      return xOffsetHighGoal;
   }
   
   public boolean seesLift(){
      return seesLift;
   }
   
   public double getxOffsetLift() {
      return xOffsetLift;
   }
   
   public double getzOffsetLift() {
      return zOffsetLift;
   }
   
   public double getPsiLift(){
      return psiLift;
   }
      
   public double getThetaLift(){
      return thetaLift;
   }
}