// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class limeyImproved extends SubsystemBase {
  /** Creates a new limeyImproved. */
  private RawFiducial[] fiducials;
  private Pose3d apriltagsDetected;
  
  public limeyImproved() {
  }

  @Override
  //Attempt at getting the raw fiducials from the limelight, aka the raw data from each individual apriltag
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("");
    apriltagsDetected = LimelightHelpers.getCameraPose3d_TargetSpace("");
    Pose2d pose = getMegaTagPose();
    if (pose != null) {
      System.out.print("Mega Tag Pose: " + pose);
    }
  }

  public RawFiducial getFiducial(int id) {
    for (RawFiducial fiducial : fiducials) {
      if (fiducial.id == id) {
        return fiducial;
      }
    }
    return null;
  }

  public Pose2d getMegaTagPose(){
    double[] pose = LimelightHelpers.getBotPose_wpiBlue("");
    if (pose.length < 6) {
      return null;
    } 

    double x = pose[0]; // X Pos
    double y = pose[1]; //Y Pos
    double rot = pose[5]; // Yaw

    return new Pose2d(x, y, Rotation2d.fromDegrees(rot));
  }

  public RawFiducial getFiducial() {
    if (fiducials.length == 0) {
      return null;
    }
    return fiducials[0]; 
}
  public Pose3d getAprilTagValues(){
    return apriltagsDetected;
  }

  public boolean apriltagDetected(){
    return apriltagsDetected != null;
  }
   
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
