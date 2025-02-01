// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.limeyImproved;

import java.util.concurrent.Semaphore;
import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsys swerve;
  //Change PID values to tune the PID loop
  private final PIDController xPID = new PIDController(0.1, 0.0, 0.0);
  private final PIDController yPID = new PIDController(0.1, 0.0, 0.0);
  private final PIDController rotPID = new PIDController(0.1, 0.0, 0.0);
    //Reason for Supplier<Pose2d> is b/c of simple testing, and could be used later if necessary.
    //Dont worry because it does NOT have to be accurate.
  /**
   * Creates a new AlignAprilTag.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignAprilTag(SwerveSubsys subsystem) {
    swerve = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   Pose2d initialPose = new Pose2d();
   if (LimelightHelpers.getBotPose2d("") == null){
    
   }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopMods();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Finished when all PIDs are at setpoint
    return xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint() && hasStarted;
  }
}
