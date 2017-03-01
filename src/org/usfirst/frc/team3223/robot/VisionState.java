package org.usfirst.frc.team3223.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.IRemote;
import edu.wpi.first.wpilibj.tables.IRemoteConnectionListener;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;

public class VisionState implements ITableListener, IRemoteConnectionListener {
   private boolean seesHighGoal;
   private double xPixelOffsetHighGoal;
   private double zPixelOffsetHighGoal;
   
   private boolean seesLift;
   private double xOffsetLift; //in mm
   private double zOffsetLift; //in mm
   private double thetaLift; //in rads - angle between robot dir and lift location
   private double psiLift;//in rads - angle between robot dir and wall (perbendicular)
   private NetworkTable networkTable;
   private int countConnected = 0;
   private int countDisconnected = 0;
	
   public VisionState() {
      networkTable = NetworkTable.getTable("SmartDashboard");
      networkTable.addTableListener(this, true);
      networkTable.addConnectionListener(this, true);
   }

   @Override
   public void valueChanged(ITable source, String key, Object value, boolean isNew) {
      if(key.equals("seesHighGoal")) {
         seesHighGoal = (boolean) value;
      }
      if(key.equals("xPixelOffsetHighGoal")) {
         xPixelOffsetHighGoal = (double) value;
      }
      if(key.equals("zPixelOffsetHighGoal")){
    	  zPixelOffsetHighGoal = (double) value;
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
	
   public double getxPixelOffsetHighGoal() {
      return xPixelOffsetHighGoal;
   }
   
   public double getzPixelOffsetHighGoal(){
	   return zPixelOffsetHighGoal;
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

@Override
public void connected(IRemote remote) {
	// TODO Auto-generated method stub
	countConnected++;
	networkTable.putString("connected", countConnected + " " + remote.toString());
	
}

@Override
public void disconnected(IRemote remote) {
	// TODO Auto-generated method stub
	countDisconnected++;
	networkTable.putString("disconnected", countDisconnected + " " + remote.toString());
	
}
}