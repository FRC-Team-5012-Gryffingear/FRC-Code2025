// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class limelightSubsystem extends SubsystemBase {
  /** Creates a new limelightSubsystem. */
  private final NetworkTable limelightTable;
  public limelightSubsystem() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight-senior");
    limelightTable.getEntry("Closest").setNumber(0);
  }

  public boolean aprilTagsExist(){
    return limelightTable.getEntry("tv").getDouble(0) == 1;
  }

  public int getAprilTagID(){
    return (int) limelightTable.getEntry("tid").getDouble(-1);
  }

  public double[] getBotPoseBlue(){
    return limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double [6]);
  }

  public Pose2d getAprilTagPoseWOffset(double xOffset, double yOffset){
    double[] botPose = getBotPoseBlue();
    Pose2d basePose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
    Transform2d offset = new Transform2d(xOffset, yOffset, new Rotation2d());
    basePose.transformBy(offset);
    return basePose;
  }

  public Pose2d getAprilTagPose(){
    double[] botPose = getBotPoseBlue();
    Pose2d basePose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
    return basePose;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
