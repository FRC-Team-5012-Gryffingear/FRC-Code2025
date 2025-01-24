// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.limeyImproved;

import java.util.concurrent.Semaphore;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsys swerve;
  private final limeyImproved limelight;
  //Change PID values to tune the PID loop
  private final PIDController xPID = new PIDController(0.03, 0, 0.0);
  private final PIDController yPID = new PIDController(0.1, 0, 0.0);
  private final PIDController rotPID = new PIDController(0.01, 0, 0.0);
  //Following two are for the THIRD Version.
  // private boolean initialized = false;
  private static boolean needs_movement = false;
  private static double initial_gyro_yaw = -10000;
  private static boolean target_seen = true;
  private static double get_Val_X = 0;
  private static double startX = 0;
  private static double desiredX = 0;

  private static double get_Val_Z = 0;
  private static double xOffset = 0;
  private static double zOffset = 0;

  private double x2, april_tag_rotation, rot2;
  /**
   * Creates a new AlignAprilTag.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignAprilTag(SwerveSubsys subsystem, limeyImproved lime) {
    swerve = subsystem;
    limelight = lime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, lime);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // xPID.setTolerance(0.1);
    // yPID.setTolerance(0.1);
    // rotPID.setTolerance(3);
    xOffset = 0;
    zOffset = 0;
    swerve.resetHeading();
    yPID.setSetpoint(.3048);
    x2 = -10000;
    april_tag_rotation = -10000;
    get_Val_X = 0;
    get_Val_Z = 0;
    rot2 = -10000;
    target_seen = false;
    swerve.resetPose();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("REAL TIME SWERVE YAW", swerve.getYaw());

    Pose3d detectedID = limelight.getAprilTagValues();
    Pose2d robotPose = swerve.getPose();

    SmartDashboard.putNumber("REAL TIME APRIL TAG", Math.toDegrees(detectedID.getRotation().getY()));

    if(LimelightHelpers.getTV("")){
      target_seen = true;
    }
    
    if(!target_seen){
      swerve.drive3(0, 0, 0, true);
    }
    else{
      if(!needs_movement){
        needs_movement = true;
        xOffset = detectedID.getX();
        zOffset = Math.abs(detectedID.getZ());
        startX = robotPose.getY();
        desiredX = startX + xOffset;
      }
      double xPower = xPID.calculate(swerve.getPose().getY(), desiredX);
      double yPower = yPID.calculate(zOffset, 0);

      if(xPower > 0.6){
        xPower = 0.6;
        System.out.println("TOOO MUCH");
      }
      if(xPower < -0.6){
        xPower = -0.6;
        System.out.print("-TOOOO MUCH");
      }
      if(Math.abs(xPower) > 0.01){
        swerve.drive3(0, -xPower, 0, false);
      } else{
        System.out.println("Perfected X");
      }
      SmartDashboard.putNumber("Horizontal Offset", xOffset);
      SmartDashboard.putNumber("Robot Horizontal Pose", startX);
      SmartDashboard.putNumber("Desired Horizontal Pose", desiredX);
      SmartDashboard.putNumber("Horizontal Power", -xPower);
      SmartDashboard.putNumber("RobotPose", swerve.getPose().getY());
      
      }
      
     

    



    // //Alternative: RawFiducial detectedID = limelight.getFiducial(X); where X is the ID of the apriltag
    // //Everything could be placed in a function onwards.
    // // RawFiducial detectedID = limelight.getFiducial();
    // Pose3d detectedID = limelight.getAprilTagValues();
    // if (LimelightHelpers.getTV("")) {
    //   // these values are in meters
    //     // double x = detectedID.getX();
    //     // double y = -detectedID.getZ();
    //     // double rot = detectedID.getRotation().getY(); //May need tweaking to get the right value
    //     // double xOutput = xPID.calculate(x, 0);
    //     // double yOutput = yPID.calculate(y, 0.3048);
    //     // double rotOutput = rotPID.calculate(rot, 0);
    //     // swerve.drive3(xOutput, yOutput, rotOutput, false);

    //     // if(xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint()) {
    //     //     swerve.drive3(0, 0, 0, false); //Change field relative if necessary.
    //     //   }
    //     // SmartDashboard.putNumber("X", x);
    //     // SmartDashboard.putNumber("Y", y);
    //     // SmartDashboard.putNumber("Rot", Math.toDegrees(rot));
    //     // SmartDashboard.putNumber("X Output", xOutput);
    //     // SmartDashboard.putNumber("Y Output", yOutput);
    //     // SmartDashboard.putNumber("Rot Output", rotOutput);


    //     // //Alternative way of getting the x, y, and rot values from the limelight but w/o while loop. THIRD
    //     if(!initialized){
    //         x2 = detectedID.getX();
    //         april_tag_rotation = detectedID.getZ();
    //         rot2 = detectedID.getRotation().getY(); //May need tweaking to get the right value
    //         xPID.reset();
    //         yPID.reset();
    //         rotPID.reset();
    //         initialized = true;

    //     }
    //     //May need to cap values?
    //     double yPos1 = yPID.getError();
    //     // double xPos1 = x2 - xPID.getAccumulatedError();
    //     // double rotPos1 = rot2 - rotPID.getAccumulatedError();
    //     // double xOutput2 = xPID.calculate(xPos1, 0);
    //     double yOutput2 = yPID.calculate(yPos1);
    //     // double rotOutput2 = rotPID.calculate(rotPos1, 0);
    //     swerve.drive3(0, yOutput2, 0, false); //Change field relative if necessary.
        
    //     //IMPORTANT NOTE, may need to set setpoints earlier?
    //     if(yPID.atSetpoint()) {
    //         swerve.drive3(0, 0, 0, false); //Change field relative if necessary.
    //          //This will cause errors, discuss
    //       }

    //     SmartDashboard.putNumber("X", x2);
    //     SmartDashboard.putNumber("Y", april_tag_rotation);
    //     SmartDashboard.putNumber("Z", detectedID.getZ());
    //     SmartDashboard.putNumber("Rot", rot2);
    //     SmartDashboard.putBoolean("Initialized Val", initialized);
    //     // SmartDashboard.putNumber("X Output", xOutput2);
    //     SmartDashboard.putNumber("Y Output", yOutput2);
    //     // SmartDashboard.putNumber("Rot Output", rotOutput2);
    //     SmartDashboard.putNumber("AccumulatedXError", xPID.getError());
    //     // SmartDashboard.putNumber("AccumulatedYError", yPID.getAccumulatedError());
    //     // SmartDashboard.putNumber("AccumulatedRotError", rotPID.getAccumulatedError());
    //     // SmartDashboard.putNumber("Theoretical X Remaining", xPos1);
    //     SmartDashboard.putNumber("Theoretical Y Remaining", yPos1);
    //     // SmartDashboard.putNumber("Theoretical Rot Remaining", rotPos1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    x2 = -10000;
    april_tag_rotation = -10000;
    
    rot2 = -10000;
    get_Val_Z = 0;
    get_Val_X = 0;
    initial_gyro_yaw = -10000;
    target_seen = false;
    needs_movement = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
