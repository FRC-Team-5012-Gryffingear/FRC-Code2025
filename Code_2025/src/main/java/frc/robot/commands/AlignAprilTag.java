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
    private Supplier<Pose2d> targetPoseSupplier;
    private boolean atX, atY, atRot;
    private boolean hasStarted = false;
  /**
   * Creates a new AlignAprilTag.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignAprilTag(SwerveSubsys subsystem, Supplier<Pose2d> targetPoseSup) {
    swerve = subsystem;
    targetPoseSupplier = targetPoseSup;
    //Sets rot PID to be continuous and sets tolerance to 0.25.
    rotPID.setTolerance(0.25);
    rotPID.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Resets all PID controllers
    xPID.reset();
    yPID.reset();
    rotPID.reset();
    //Sets Tolerance (Error) for PIDs and sets atX, atY, atRot to false
    xPID.setTolerance(0.5);
    yPID.setTolerance(0.5);
    atX = atY = atRot = hasStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Creates a target pose and robot pose.
    Pose2d targetPose = targetPoseSupplier.get();
    Pose2d robotPose = swerve.getPose();
    //Gets the mega tag pose from the limelight...
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
    //Gets mega tag pose (Which is why targetPoseSupplier may be unnecessary)
    if(mt2.tagCount >0){
      hasStarted = true;
      targetPose = new Pose2d(mt2.pose.getTranslation(), mt2.pose.getRotation());
    
    //Sets the setpoint for the PID controllers
    xPID.setSetpoint(targetPose.getX());
    yPID.setSetpoint(targetPose.getY());
    rotPID.setSetpoint(targetPose.getRotation().getRadians());

    //Calculates the output for the PID Controllers
    double xOut = xPID.calculate(robotPose.getX());
    double yOut = yPID.calculate(robotPose.getY());
    //Its complicated because it would need to flipped if it was red alliance.
    double rotOut = rotPID.calculate(robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians() + (DriverStation.getAlliance().isPresent()
      && DriverStation.getAlliance().get() == Alliance.Red
        ? Math.PI
          : 0.0));
    
    //Cancels power... if at setpoint
    if(xPID.atSetpoint() || atX){
      xOut = 0;
    }

    if(yPID.atSetpoint() || atY){
      yOut = 0;
    }

    if(rotPID.atSetpoint() || atRot){
      rotOut = 0;
    }

    //Unused boolean.
    boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

    //Drives the robot using the PID outputs
    swerve.drive3(xOut, yOut, rotOut, true);
    } 
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
