// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intakeCombined;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.limelightSubsystem;





/** An example command that uses an example subsystem. */
public class AprilTagAlignment extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ScoreCoralComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AprilTagAlignment(SwerveSubsys swerve, intakeCombined intake, ElevatorSubsys elev, limelightSubsystem lime, double xOffset) {
    if(lime.aprilTagsExist()){
        Pose2d targetpose = lime.getAprilTagPoseWOffset(xOffset, 0);
        Command pathCommand = AutoBuilder.pathfindToPose(targetpose, new PathConstraints(16.497, 3, 2378.027, 720.000));
        pathCommand.schedule();
    } 
  }
}

