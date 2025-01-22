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

import edu.wpi.first.math.controller.PIDController;
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
  private final PIDController xPID = new PIDController(0.1, 0.1, 0.0);
  private final PIDController yPID = new PIDController(0.1, 0.1, 0.0);
  private final PIDController rotPID = new PIDController(0.1, 0.1, 0.0);
  //Following two are for the THIRD Version.
  // private boolean initialized = false;
  private boolean needs_rotate = false;
  private double initial_gyro_yaw = -10000;
  private boolean target_seen = true;

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
    yPID.setSetpoint(.3048);
    x2 = -10000;
    april_tag_rotation = -10000;
    rot2 = -10000;
    target_seen = false;

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Pose3d detectedID = limelight.getAprilTagValues();
    
    // if(!target_seen){
    //   swerve.drive3(0, 0, 0.15, true);
    // }
    // else{
    //   needs_rotate = true;
    //   if(needs_rotate && april_tag_rotation == -10000 && initial_gyro_yaw == -10000){
    //     april_tag_rotation = LimelightHelpers.getCameraPose3d_TargetSpace("").getRotation().getY();
    //     initial_gyro_yaw = swerve.getYaw();
    //   }
    //   if(needs_rotate){
    //     double final_gyro_yaw = initial_gyro_yaw + april_tag_rotation;
    //     yPID.calculate(swerve.getYaw(),final_gyro_yaw);
    //     if(yPID.getError() < 0.01){
    //       needs_rotate = false;
    //       april_tag_rotation = -10000;
    //     }
    //   }

    // }

    // SmartDashboard.putNumber("Apriltag_rotation", april_tag_rotation);
    // SmartDashboard.putBoolean("needs rotate", needs_rotate);


    // // //Alternative: RawFiducial detectedID = limelight.getFiducial(X); where X is the ID of the apriltag
    // // //Everything could be placed in a function onwards.
    // // // RawFiducial detectedID = limelight.getFiducial();
    // // Pose3d detectedID = limelight.getAprilTagValues();
    // // if (LimelightHelpers.getTV("")) {
    // //   // these values are in meters
    // //     // double x = detectedID.getX();
    // //     // double y = -detectedID.getZ();
    // //     // double rot = detectedID.getRotation().getY(); //May need tweaking to get the right value
    // //     // double xOutput = xPID.calculate(x, 0);
    // //     // double yOutput = yPID.calculate(y, 0.3048);
    // //     // double rotOutput = rotPID.calculate(rot, 0);
    // //     // swerve.drive3(xOutput, yOutput, rotOutput, false);

    // //     // if(xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint()) {
    // //     //     swerve.drive3(0, 0, 0, false); //Change field relative if necessary.
    // //     //   }
    // //     // SmartDashboard.putNumber("X", x);
    // //     // SmartDashboard.putNumber("Y", y);
    // //     // SmartDashboard.putNumber("Rot", Math.toDegrees(rot));
    // //     // SmartDashboard.putNumber("X Output", xOutput);
    // //     // SmartDashboard.putNumber("Y Output", yOutput);
    // //     // SmartDashboard.putNumber("Rot Output", rotOutput);


    // //     // //Alternative way of getting the x, y, and rot values from the limelight but w/o while loop. THIRD
    // //     if(!initialized){
    // //         x2 = detectedID.getX();
    // //         april_tag_rotation = detectedID.getZ();
    // //         rot2 = detectedID.getRotation().getY(); //May need tweaking to get the right value
    // //         xPID.reset();
    // //         yPID.reset();
    // //         rotPID.reset();
    // //         initialized = true;

    // //     }
    // //     //May need to cap values?
    // //     double yPos1 = yPID.getError();
    // //     // double xPos1 = x2 - xPID.getAccumulatedError();
    // //     // double rotPos1 = rot2 - rotPID.getAccumulatedError();
    // //     // double xOutput2 = xPID.calculate(xPos1, 0);
    // //     double yOutput2 = yPID.calculate(yPos1);
    // //     // double rotOutput2 = rotPID.calculate(rotPos1, 0);
    // //     swerve.drive3(0, yOutput2, 0, false); //Change field relative if necessary.
        
    // //     //IMPORTANT NOTE, may need to set setpoints earlier?
    // //     if(yPID.atSetpoint()) {
    // //         swerve.drive3(0, 0, 0, false); //Change field relative if necessary.
    // //          //This will cause errors, discuss
    // //       }

    // //     SmartDashboard.putNumber("X", x2);
    // //     SmartDashboard.putNumber("Y", april_tag_rotation);
    // //     SmartDashboard.putNumber("Z", detectedID.getZ());
    // //     SmartDashboard.putNumber("Rot", rot2);
    // //     SmartDashboard.putBoolean("Initialized Val", initialized);
    // //     // SmartDashboard.putNumber("X Output", xOutput2);
    // //     SmartDashboard.putNumber("Y Output", yOutput2);
    // //     // SmartDashboard.putNumber("Rot Output", rotOutput2);
    // //     SmartDashboard.putNumber("AccumulatedXError", xPID.getError());
    // //     // SmartDashboard.putNumber("AccumulatedYError", yPID.getAccumulatedError());
    // //     // SmartDashboard.putNumber("AccumulatedRotError", rotPID.getAccumulatedError());
    // //     // SmartDashboard.putNumber("Theoretical X Remaining", xPos1);
    // //     SmartDashboard.putNumber("Theoretical Y Remaining", yPos1);
    // //     // SmartDashboard.putNumber("Theoretical Rot Remaining", rotPos1);

        
        
        
        
        
        
        
    // //     //Get the x, y, and rot values from the limelight using static variables. SECOND
    // //     // double x1 = detectedID.getX();
    // //     // double rot1 = detectedID.getRotation().getY(); //May need tweaking to get the right value
    // //     // double absDistance = detectedID.getTranslation().getZ();
    // //     // double forwardDistance = Math.sqrt(Math.pow(absDistance, 2) * Math.pow(x1, 2));
    // //     // double y1 = forwardDistance;
    // //     // yPID.reset();
    // //     // xPID.reset();
    // //     // rotPID.reset();
    // //     // System.out.println("AAAAAAAAAAAAAAAAAAAAAAAA");
    // //     // while(!(xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint())){
    // //     //     //May need to cap values?
    // //     //     System.out.println("Hey man ");
    // //     //     xPID.setSetpoint(0);
    // //     //     rotPID.setSetpoint(0);
    // //     //     yPID.setSetpoint(0.3408);
    // //     //     double yPos = y1 - yPID.getAccumulatedError();
    // //     //     double xPos = x1 - xPID.getAccumulatedError();
    // //     //     double rotPos = rot1 - rotPID.getAccumulatedError();
    // //     //     double xOutput1 = xPID.calculate(xPos, 0);
    // //     //     double yOutput = yPID.calculate(yPos, 0.3408);
    // //     //     double rotOutput1 = rotPID.calculate(rotPos, 0);
    // //     //     swerve.drive3(xOutput1, yOutput, rotOutput1, false); //Change field relative if necessary.
    // //     //     //Hypothetical way of decreasing x1, y1, rot1 to decrease themselves.
    // //     //     // x1 -= xOutput1;
    // //     //     // y1 -= yOutput1;
    // //     //     // rot1 -= rotOutput1;
    // //     //     SmartDashboard.putNumber("X", x1);
    // //     //     SmartDashboard.putNumber("Y", y1);
    // //     //     SmartDashboard.putNumber("Y Ouput", yOutput);
    // //     //     SmartDashboard.putNumber("The Distance Z", detectedID.getZ());
    // //     //     SmartDashboard.putNumber("Rot", rot1);
    // //     //     SmartDashboard.putNumber("X Output", xOutput1);
    // //     //     SmartDashboard.putNumber("Rot Output", rotOutput1);

    // //     // }
    // //     // System.out.println("THe PID Condition has been met.");
        







    // //     //Get the x, y, and rot values from the limelight with continous movement. FIRST
    // //     // double x = detectedID.getX();
    // //     // double y = detectedID.getZ();
    // //     // double rot = detectedID.getRotation().getY(); //May need tweaking to get the right value
    // //     // double xOutput = xPID.calculate(x, 0);
    // //     // double yOutput = yPID.calculate(y, 0.5);
    // //     // double rotOutput = rotPID.calculate(rot, 0);
    // //     // swerve.drive3(xOutput, yOutput, rotOutput, false);
    // //     // if(xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint()) {
    // //     //     swerve.drive3(0, 0, 0, false); //Change field relative if necessary.
    // //     //   }
    // //     // SmartDashboard.putNumber("X", x);
    // //     // SmartDashboard.putNumber("Y", y);
    // //     // SmartDashboard.putNumber("Rot", rot);
    // //     // SmartDashboard.putNumber("X Output", xOutput);
    // //     // SmartDashboard.putNumber("Y Output", yOutput);
    // //     // SmartDashboard.putNumber("Rot Output", rotOutput);

    // // } else {
    // //     swerve.drive3(0, 0, 15, false);
    // // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
