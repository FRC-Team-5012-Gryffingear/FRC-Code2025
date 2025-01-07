// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limey extends SubsystemBase {
  /** Creates a new limey. */
  public double kp = .02;
  PIDController movement = new PIDController(kp, 0, 0);



  public limey() {

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
  public double rotationLock(double x_value){
    double rot = movement.calculate(x_value,0);
    if(Math.abs(rot) < .15){
      rot = 0;
    }
    SmartDashboard.putNumber("Rotational power", -rot);
    return -rot;
  }



//verified working
  public double fwrdLock(double z_value){

    // Calculates the speed needed to reach goal
    double zPower = movement.calculate(z_value,15); 
    

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
    double xPower = movement.calculate(x_value, 0);
    
    // caps the motor powers on an interval of [-1,1]
    xPower = Math.max(-1, Math.min(1,xPower));
    
    SmartDashboard.putNumber("X power input", -xPower);

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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}