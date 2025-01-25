// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.SwerveMod;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.limeyImproved;

import java.util.concurrent.Semaphore;

import org.opencv.core.Mat;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsys swerve;
  private final limeyImproved limelight;
  //Change PID values to tune the PID loop
  private final PIDController xPID = new PIDController(0.1, 0, 0.0);
  private final PIDController yPID = new PIDController(0.1, 0, 0.0);
  private final PIDController rotPID = new PIDController(0.01, 0, 0.0);
  //Following two are for the THIRD Version.
  // private boolean initialized = false;
  private static boolean needs_rotate = false;
  private static double initial_gyro_yaw = -10000;
  private static boolean target_seen = true;
  private static double get_Val_X = 0;
  private static double get_Val_Z = 0;
  private static double get_TX = 0;
  private static double final_gyro_yaw = 1000000;
  private static boolean look_for_new_tag = true;

  // Math to convert rotation into distance 2pi * r (r will be in meters to match Z)
  private static double distance_per_rot = Units.inchesToMeters(2) * Math.PI * 2; 


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
    look_for_new_tag = true;
    final_gyro_yaw = 1000000;
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
    // double real_wheel_rotation = swerve.odometry.getPoseMeters().getX();

    SmartDashboard.putNumber("REAL TIME SWERVE YAW", swerve.getYaw());

    // SmartDashboard.putNumber("Swerve movement X value", real_wheel_rotation);

    Pose3d detectedID = limelight.getAprilTagValues();

    SmartDashboard.putNumber("REAL TIME APRIL TAG", Math.toDegrees(detectedID.getRotation().getY()));

    if(LimelightHelpers.getTV("")){
      target_seen = true;
    }
    
    if(!target_seen){
      swerve.drive3(0, 0, 0, true);
    }
    else{
     while(Math.abs(final_gyro_yaw - swerve.getYaw()) > 0.1 && look_for_new_tag){ 
      needs_rotate = true;
      SmartDashboard.putNumber("LIVE FEED APRIL TAG", LimelightHelpers.getCameraPose3d_TargetSpace("").getY());
      SmartDashboard.putBoolean("needs rotate", needs_rotate);

      if(needs_rotate && initial_gyro_yaw == -10000){ //  && april_tag_rotation == -10000
        // april_tag_rotation = Math.toDegrees(LimelightHelpers.getCameraPose3d_TargetSpace("").getY());
        initial_gyro_yaw = swerve.getYaw();
        // get_Val_X = detectedID.getX();
        get_Val_Z = detectedID.getZ();
        get_Val_X = detectedID.getX();
        april_tag_rotation = detectedID.getRotation().getY();
        // final_gyro_yaw = Math.toDegrees(april_tag_rotation) + initial_gyro_yaw;
        

        
        // april_tag_rotation = -Math.toDegrees(LimelightHelpers.getCameraPose3d_TargetSpace("").getY());
      }
      if(needs_rotate){
        //Right side of april tag is negative, left side of april tag is positive
        // Double the angle to "predict" and might need to remove negative
        //  double final_gyro_yaw = -Math.toDegrees(Math.atan(get_Val_X/ get_Val_Z));
        //  double final_angle = (final_gyro_yaw * 2) + initial_gyro_yaw;
        
         // USE limelight TX to rotate amount necessary

         SmartDashboard.putNumber(("GET X"), get_Val_X);
         SmartDashboard.putNumber("GET Z", get_Val_Z);
         final_gyro_yaw = Math.toDegrees(april_tag_rotation) + initial_gyro_yaw;
        SmartDashboard.putNumber("Final_gyro_value", final_gyro_yaw);
        // arctan(x/z)
        // total = 180 - apriltagyaw
        // AFTER do total2 = 90 - total
        // final gyro =  total2
        // SmartDashboard.putNumber("Final Gyro addition BEFORE", final_angle);
        // SmartDashboard.putNumber("ANGLE THETA DOS", math_4_angle);
        // SmartDashboard.putNumber("Error thresh", Error_thresh);
        double speed = rotPID.calculate(swerve.getYaw(), -final_gyro_yaw);
        SmartDashboard.putNumber("speeedd BEFORE", speed);

        SmartDashboard.putNumber("Math Threshold", Math.abs(final_gyro_yaw - swerve.getYaw()));
        if(Math.abs(final_gyro_yaw - swerve.getYaw()) < 0.1){
          speed = 0;
          needs_rotate = false;
        }

        swerve.drive3(0, 0, speed, false);
        SmartDashboard.putNumber("speeedd AFTER", speed);
        // swerve.drive3(0, 0, speed, false);
        
        
      
      }

      SmartDashboard.putNumber("April tag rotation", april_tag_rotation);
      SmartDashboard.putNumber("Init gyro yaw", initial_gyro_yaw);

     }
     final_gyro_yaw = 1000000;
     look_for_new_tag = false;
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
    needs_rotate = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
