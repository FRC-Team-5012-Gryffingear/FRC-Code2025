// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.LedSubsystem;
// import frc.robot.subsystems.VisionSub;
import frc.robot.subsystems.SwerveSubsys;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.limey;

/** An example command that uses an example subsystem. */
public class SwerveCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsys swerve;
 private final CommandXboxController controller2;
 private final BooleanSupplier yaw;

 //Auto
 private double initial_gyro_yaw, get_Val_X, get_Val_Z, april_tag_rotation, abs_final_Rot;

 private double final_gyro_yaw, speedY, speedZ, rotspeed, store_auto_yaw;


 private boolean phase1, phase2, seenTag = false;
 private final PIDController xPID = new PIDController(0.1, 0, 0.0);
 private final PIDController yPID = new PIDController(0.6, 0, 0.0);
 private final PIDController rotPID = new PIDController(0.01, 0, 0.0);
 private final PIDController auto_yaw = new PIDController(.01, 0, 0.0);
  // private final BooleanSupplier yaw;

  // private final BooleanSupplier limeLock;

  //private final LedSubsystem Ledsubsys; 
  public double t = 1;


 
  
  public SwerveCom(SwerveSubsys subsystem, CommandXboxController controller, BooleanSupplier yaw) {
    swerve = subsystem;
    controller2 = controller;
    this.yaw = yaw;
    
  
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //vision.startThread();
    swerve.resetPose();
    swerve.resetHeading();
    phase1 = false;
    phase2 = false;
    seenTag = false;
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = MathUtil.applyDeadband(controller2.getLeftY(), OperatorConstants.Deadband) * 0.4;
    double ySpeed = MathUtil.applyDeadband(controller2.getLeftX(), OperatorConstants.Deadband) * 0.4;
    double rotateSpeed = MathUtil.applyDeadband(controller2.getRightX(), OperatorConstants.Deadband) * 0.4;
    SmartDashboard.putNumber("YAW YAW YAW", swerve.getYaw());
    if(yaw.getAsBoolean()){
      swerve.resetHeading();
    }



    //swerve.drive3(xSpeed, -ySpeed, -rotateSpeed*1.5, true);
    SmartDashboard.putBoolean("SeenTag", seenTag);
    SmartDashboard.putBoolean("TelePhase1", phase1);
    SmartDashboard.putBoolean("TelePhase2", phase2);
    if(controller2.b().getAsBoolean() && seenTag){
      if(phase1){
        phase1();
      }
      if(phase2){
        phase2();
      }
    
    } else{
      resetValues();
      if(LimelightHelpers.getTV("")){
        seenTag = true;
        updateAprilTagValues();
      } else{
        seenTag = false;
      }
      swerve.drive3(xSpeed, -ySpeed, -rotateSpeed*1.5, true);
    }
    getGlobalInfo();

    /* This is assuming we are storing data first then moving 
    if(button pressed/held){
       phase1();
       phase2();
      } else{
        resetValues();
        swerve.drive(inputs); 
        }

        public void phase1(){
        // Copy Code here
        }

        public void phase2(){
        // Copy Code here
        }

        public resetValues(){
        //Copy Code here
        }
     */




    SmartDashboard.putNumber("getXXValue", LimelightHelpers.getCameraPose3d_TargetSpace("").getX());
    SmartDashboard.putNumber("side value", (swerve.odometry.getPoseMeters().getY()/14.968) -.2);
    SmartDashboard.putNumber("fwd value", (swerve.odometry.getPoseMeters().getX()/14.968) -.2);
  }


  public void phase1(){
    final_gyro_yaw = Math.toDegrees(april_tag_rotation) + initial_gyro_yaw;
    if(april_tag_rotation > 0){
      abs_final_Rot = Math.copySign(Math.abs(final_gyro_yaw-3), final_gyro_yaw);
    }
    else{
      abs_final_Rot = Math.copySign(Math.abs(final_gyro_yaw+10), final_gyro_yaw);
    }
    rotspeed = MathUtil.clamp(rotPID.calculate(swerve.inv_get_Yaw(),abs_final_Rot), -.5, .5);
    if(Math.abs(rotspeed) < 0.05){
      rotspeed = 0;
      phase1 = false;
      phase2 = true;
      swerve.resetPose();
      return;
    }    
    swerve.drive3(0, 0, -rotspeed, false);
  }

  public void phase2(){
    double conversion = 14.968;
    double abs_final_Y = 0;
      
//may not work
      if( (-get_Val_X) > 0){
       abs_final_Y = Math.copySign((Math.abs(get_Val_X) + (0.196 / get_Val_X)), -get_Val_X);
      }
      else{
        abs_final_Y = Math.copySign(Math.abs(get_Val_X) + (0.549 / get_Val_X ), -get_Val_X);
      }

      speedY = MathUtil.clamp(yPID.calculate(((swerve.odometry.getPoseMeters().getY()) / conversion), abs_final_Y) , -.04,.04); // -get_Val_X as setpoint
      speedZ = MathUtil.clamp(xPID.calculate((swerve.odometry.getPoseMeters().getX() / conversion)+(1.1 * get_Val_Z / 1.79),-get_Val_Z), -0.05, 0.05);// -.78
      store_auto_yaw = MathUtil.clamp(auto_yaw.calculate(swerve.inv_get_Yaw(),abs_final_Rot),-.2,.2);

      SmartDashboard.putNumber("Current Side Position", ((swerve.odometry.getPoseMeters().getY()) / conversion));
      SmartDashboard.putNumber("Current Forward Position", (swerve.odometry.getPoseMeters().getX() / conversion)+(1.1 * get_Val_Z / 1.79));
      SmartDashboard.putNumber("Goal Side", abs_final_Y);
      SmartDashboard.putNumber("Goal Forward", -get_Val_Z);


      if(Math.abs(speedZ) < 0.03 && Math.abs(speedY) < 0.01){
        speedZ = 0; 
        speedY = 0;
        phase2 = false;
      }

      // swerve.drive3(0,-speedY, 0, false);

      swerve.drive3(-speedZ, -speedY, -store_auto_yaw, false);
  }

  public void updateAprilTagValues(){
    Pose3d detectedID = LimelightHelpers.getCameraPose3d_TargetSpace("");
    initial_gyro_yaw = swerve.getYaw();
    get_Val_Z = detectedID.getZ();
    get_Val_X = detectedID.getX();
    april_tag_rotation = detectedID.getRotation().getY();

    SmartDashboard.putNumber("LastSeenX", get_Val_X);
    SmartDashboard.putNumber("LastSeenZ", get_Val_Z);
    SmartDashboard.putNumber("LastSeenRot", april_tag_rotation);
    SmartDashboard.putNumber("initialGyroYaw", initial_gyro_yaw);
  }

  public void resetValues(){
    phase1 = true;
    phase2 = false;
    final_gyro_yaw = 0;
    speedY = 0;
    speedZ = 0;
    rotspeed = 0;
    store_auto_yaw = 0;
  }

  public void getGlobalInfo(){
    SmartDashboard.putNumber("TeleSideSpeedPhase2", -speedY);
    SmartDashboard.putNumber("TeleForwardSpeedPhase2", -speedZ);
    SmartDashboard.putNumber("TeleRotSpeedPhase2", -store_auto_yaw);
    SmartDashboard.putNumber("TeleFinalGyro", final_gyro_yaw);
    SmartDashboard.putNumber("TeleRotSpeedPhase1", rotspeed);
    SmartDashboard.putNumber("SwerveYawTeleOp", swerve.inv_get_Yaw());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopMods();
    resetValues();
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}