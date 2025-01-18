// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limey extends SubsystemBase {
  /** Creates a new limey. */
  public double kp = .02;
  PIDController movement = new PIDController(kp, 0, 0);
  PIDController sideContrl = new PIDController(.01, 0, 0);



  public limey() {
    LimelightHelpers.setPipelineIndex("Closest", 0);
  }

  public double getX(){
    return LimelightHelpers.getTX("");
  }
  public double getY(){
    return LimelightHelpers.getTY("");
  }

  public double estimate3DZInches(){
    double distance = getTZ();
    double InchesInOneZ = 39.6153846154;
    double distanceInches = distance * InchesInOneZ;
    return distanceInches;
    //Max Distance is 199 inches for higher quality camera
  }
  
  public double getTZ(){
    Pose3d poses = LimelightHelpers.getTargetPose3d_CameraSpace("");
    return poses.getZ();
  }





// verified working
// Controls the steering/turning
// locks onto apriltag once to prevent constant reading
  public double rotationLock(double x_value){
    double rot = movement.calculate(x_value,0);
    if(Math.abs(rot) < .15){
      rot = 0;
    }

    rot = Math.max(-1, Math.min(1,rot));
    SmartDashboard.putNumber("Rotational power", -rot);
    return -rot;
  }



//verified working
  public double fwrdLock(double z_value){

    // Calculates the speed needed to reach goal
    double zPower = movement.calculate(z_value,30); 
    

    // caps the motor powers on an interval of [-1,1]
    zPower = Math.max(-1, Math.min(1,zPower));

    if(Math.abs(zPower) < .03){
      zPower = 0;
    }

    if(getID() == -1){
      zPower = 0;
    }

    SmartDashboard.putNumber("Z power input", -zPower);
    return -zPower;
  }

// verified working
// side to side movement to get infront of the april tag
  public double rotAround(double x_value){
    // Calculates the speed needed to reach goal
    // movement.setP(.02);
    Pose3d tagP = LimelightHelpers.getCameraPose3d_TargetSpace("");
    double xPower = 0;
    double rotVal = Math.toDegrees(tagP.getRotation().getY());
    // double pose = Math.atan(getX()/estimate3DZInches());
    if(-10 < rotVal && rotVal < 10){
      xPower = 0;
    }else{
      xPower = sideContrl.calculate(rotVal, 10);
    }
   
    if(xPower > 1){
      xPower = 1;
    }
    else if(xPower < -1){
      xPower = -1;
    }

    SmartDashboard.putNumber("xPower value" , -xPower);
    SmartDashboard.putNumber("rotational value of tag",(rotVal));
    
    return -xPower;
  }


// Check which of the 2 getID's work
  public double getID(){
    double ID_detected =  LimelightHelpers.getFiducialID("");
    SmartDashboard.putNumber("ID DETECTED IN getID", ID_detected);
    return ID_detected;
  }




  @Override
  public void periodic() { 
    SmartDashboard.putNumber("Estimated Z: ", estimate3DZInches());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}