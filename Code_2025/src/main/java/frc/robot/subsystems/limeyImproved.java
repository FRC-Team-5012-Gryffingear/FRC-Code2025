// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
  }

  public RawFiducial getFiducial(int id) {
    for (RawFiducial fiducial : fiducials) {
      if (fiducial.id == id) {
        return fiducial;
      }
    }
    return null;
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
   
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
