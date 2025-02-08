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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/** An example command that uses an example subsystem. */
public class AlignAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsys swerve;
  private final limeyImproved limelight;
  // private final String limeName = "";

  //Change PID values to tune the PID loop
  private final PIDController xPID = new PIDController(0.1, 0, 0.0);
  private final PIDController yPID = new PIDController(0.6, 0, 0.0);
  private final PIDController rotPID = new PIDController(0.01, 0, 0.0);
  private final PIDController auto_yaw = new PIDController(.01, 0, 0.0);
  //Following two are for the THIRD Version.
  // private boolean initialized = false;
  private static boolean needs_rotate = false;
  private static double initial_gyro_yaw = -10000;
  private static boolean target_seen = true;
  private static double get_Val_X = 0;
  private static double get_Val_Z = 0;
  private static double final_gyro_yaw = 1000000;
  private static boolean look_for_new_tag = true;
  private static boolean moving_fwd = false;
  private static boolean moving_side = false;
  private static boolean check, phase1,phase2 = false;
  private static final double conversion = 14.968;
  // private static double snap = 0;
  private static double abs_final = 0;


  private static double first_tag_id = -1;
  private static double current_id = -1;

  // Math to convert rotation into distance 2pi * r (r will be in meters to match Z)


  private double april_tag_rotation;
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
    check =false;
    abs_final = 0;
    moving_fwd = false;
    moving_side = false;
    look_for_new_tag = false;
    final_gyro_yaw = 1000000;
    swerve.resetHeading();
    april_tag_rotation = -10000;
    get_Val_X = 0;
    get_Val_Z = 0;
    target_seen = false;
    first_tag_id = -1;
    current_id = -1;
    phase1 = true;
    phase2 = false;

    swerve.resetPose();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double real_wheel_rotation = swerve.odometry.getPoseMeters().getX();
    System.out.println("WE ARE EXECUTING");

    SmartDashboard.putNumber("REAL TIME SWERVE YAW", swerve.getYaw());

    // SmartDashboard.putNumber("Swerve movement X value", real_wheel_rotation);

    Pose3d detectedID = limelight.getAprilTagValues();

    SmartDashboard.putNumber("REAL TIME APRIL TAG", Math.toDegrees(detectedID.getRotation().getY()));

    if(LimelightHelpers.getTV("")){
      target_seen = true;
      current_id = LimelightHelpers.getFiducialID("");
    }
    
    if(!target_seen){
      swerve.drive3(0, 0, 0, false);
    }
    else{

    // if(Math.round(swerve.inv_get_Yaw()) == Math.round(detectedID.getRotation().getY())){    }
      // SmartDashboard.putBoolean(, check)
    if(phase1){

    
      // LimelightHelpers.setLEDMode_ForceOff("");
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

        first_tag_id = LimelightHelpers.getFiducialID("");
        // final_gyro_yaw = Math.toDegrees(april_tag_rotation) + initial_gyro_yaw;
        

        
        // april_tag_rotation = -Math.toDegrees(LimelightHelpers.getCameraPose3d_TargetSpace("").getY());
      }
      if(needs_rotate){

         SmartDashboard.putNumber(("GET X"), get_Val_X);
         SmartDashboard.putNumber("GET Z", get_Val_Z);


        final_gyro_yaw = Math.toDegrees(april_tag_rotation) + initial_gyro_yaw;
        SmartDashboard.putNumber("Final_gyro_value", final_gyro_yaw+10);
        
        
        if(april_tag_rotation > 0){
          abs_final = Math.copySign(Math.abs(final_gyro_yaw-3), final_gyro_yaw);
        }
        else{
          abs_final = Math.copySign(Math.abs(final_gyro_yaw+10), final_gyro_yaw);
        }


        SmartDashboard.putNumber("Abs final val", abs_final);

        double speed = MathUtil.clamp(rotPID.calculate(swerve.inv_get_Yaw(),abs_final), -.5, .5);
        SmartDashboard.putNumber("Swerve inside", swerve.inv_get_Yaw());
        SmartDashboard.putNumber("speeedd BEFORE", speed);
        

        if(Math.abs(speed) < 0.05){ // Math.abs(final_gyro_yaw - swerve.getYaw()) < 0.5
          speed = 0;
          needs_rotate = false;
          look_for_new_tag = true;

          SmartDashboard.putNumber("speeedd AFTER INNNN", speed);

          // swerve.drive3(0, 0, -speed, false);
          // snap = swerve.getYaw();
          phase1 = false;
          phase2 = true;
        }

        swerve.drive3(0, 0, -speed, false);

        SmartDashboard.putNumber("speeedd AFTER", -speed);
      }

      SmartDashboard.putNumber("April tag rotation", april_tag_rotation);
      SmartDashboard.putNumber("Init gyro yaw", initial_gyro_yaw);
      System.out.println("INSIDE");

    }
     // End of OG While Loop
     

     System.out.println("OUTSIDE");
    







     
     SmartDashboard.putBoolean("looking for new tag", look_for_new_tag);

     if(look_for_new_tag == true){
      moving_fwd = true;
      moving_side = true;
      System.out.println("the true falser ");
     }































     if(phase2){

     SmartDashboard.putBoolean("Boolean forward", moving_fwd);
     if(moving_fwd){
      if(!check){
        swerve.resetPose();
        check = true;
      }
      double abs_final_Y = 0;

      if( (-get_Val_X) > 0){
        // -.02 makes it full front
        abs_final_Y = Math.copySign((Math.abs(get_Val_X)), -get_Val_X);// -0.02
        // abs_final = Math.copySign(Math.abs(final_gyro_yaw-3), final_gyro_yaw);
      }
      else{
        abs_final_Y = Math.copySign(Math.abs(get_Val_X)+.4, -get_Val_X); // +.17
        // abs_final = Math.copySign(Math.abs(final_gyro_yaw+10), final_gyro_yaw);
      }

      double speedY = MathUtil.clamp(yPID.calculate(((swerve.odometry.getPoseMeters().getY()) / conversion), abs_final_Y) , -.04,.04); // -get_Val_X as setpoint

      SmartDashboard.putNumber("Abs final side move", abs_final_Y);

      
      //SIDE TO SIDE: odometry Y / FOWARD BACK: Odometry X 
           

      double speedZ = MathUtil.clamp(xPID.calculate((swerve.odometry.getPoseMeters().getX() / conversion)-.9,-get_Val_Z), -0.05, 0.05);// -.78

      double store_auto_yaw = MathUtil.clamp(auto_yaw.calculate(swerve.inv_get_Yaw(),abs_final),-.2,.2);
      SmartDashboard.putNumber("Auto yaw value ", store_auto_yaw);

      SmartDashboard.putNumber("Z pose robot", swerve.odometry.getPoseMeters().getX() / conversion);
      // SmartDashboard.putNumber("New getValue Z", getNewZ);



      SmartDashboard.putNumber("get Value Z", -get_Val_Z);  
      // SmartDashboard.putNumber("XAVALUE", speedX);
      SmartDashboard.putNumber("FWD SPEED BEFORE", -speedZ);
      // (x, y, rot, false)
      
      // swerve.drive3(0,speedZ, 0,false);
      
      
      SmartDashboard.putNumber("Store auto yaw", -store_auto_yaw);
      
      SmartDashboard.putNumber("X pose robot", swerve.odometry.getPoseMeters().getY() / conversion);
      SmartDashboard.putNumber("get Value X", -get_Val_X);
      SmartDashboard.putNumber("SIDE SPEED BEFORE", -speedY);
      
     

      if(Math.abs(speedZ) < 0.01 && Math.abs(speedY) < 0.01){
        speedZ = 0; 
        speedY = 0;
        moving_side = false;
        moving_fwd = false;
        phase2 = false;
      }

      // swerve.drive3(0,-speedY, 0, false);

      swerve.drive3(-speedZ, -speedY, -store_auto_yaw, false);

      // swerve.drive3(0, 0, -store_auto_yaw,false);
      SmartDashboard.putNumber("FWD SPEED AFTER", -speedZ);
     }


    //  if(!moving_fwd && moving_side){

      
      //Side to side movement
      // abs_final_Y = Math.copySign((Math.abs(get_Val_X) - 0.02), -get_Val_X) ;



      
      // double store_auto_yaw = MathUtil.clamp(auto_yaw.calculate(swerve.inv_get_Yaw(),abs_final),-.2,.2);

      // if(Math.abs(store_auto_yaw) < 0.01){
      //   store_auto_yaw = 0;
      // }

      // swerve.drive3(0, -speedY, 0, false);



      // if(Math.abs(speedY) < 0.03){
      //   speedY = 0;
      //   moving_side = false;
      //   phase2 = false;
      // }
      // swerve.drive3(0, -speedY, -store_auto_yaw, false);

      // SmartDashboard.putNumber("SIDE SPEED AFTER", -speedY);
    //  }
    
     }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    april_tag_rotation = -10000;
    
    get_Val_Z = 0;
    get_Val_X = 0;
    initial_gyro_yaw = -10000;
    target_seen = false;
    needs_rotate = false;
    phase1 = false;
    phase2 = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}