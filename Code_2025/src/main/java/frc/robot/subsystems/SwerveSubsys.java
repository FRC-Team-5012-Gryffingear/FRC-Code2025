// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Copyright (c) 2009-2023 FIRST and other WPILib contributors All rights reserved.
 */
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveSubsys extends SubsystemBase {
 

//Using the SwerveMod file constructer we define and set the SwerveMods 
//according to their ports and name
private final SwerveMod frontLeftMod = new SwerveMod(
    Constants.FrontLeftDriveID, Constants.FrontLeftSteerID, Constants.FrontLeftEncoderID, Constants.FrontLeftOffset, Constants.FrontLeftInv, "FL");
private final SwerveMod frontRightMod = new SwerveMod(
    Constants.FrontRightDriveID, Constants.FrontRightSteerID, Constants.FrontRightEncoderID, Constants.FrontRightOffset, Constants.FrontRightInv, "FR");
private final SwerveMod backLeftMod = new SwerveMod(
    Constants.BackLeftDriveID, Constants.BackLeftSteerID, Constants.BackLeftEncoderID, Constants.BackLeftOffset, Constants.BackLeftInv, "BL");
private final SwerveMod backRightMod = new SwerveMod(
    Constants.BackRightDriveID, Constants.BackRightSteerID, Constants.BackRightEncoderID, Constants.BackRightOffset, Constants.backRightInv, "BR");

// Pigeon object that determines the yaw,roll,pitch
private final Pigeon2 pigeon = new Pigeon2(Constants.PigeonID);


//  FL, FR, BL,
// Determines robots position and heading on field
private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.kinematics, getHeading(), new SwerveModulePosition[] {
    frontLeftMod.getModPos(),
    frontRightMod.getModPos(),
    backLeftMod.getModPos(),
    backRightMod.getModPos()
});

//makes a 2d field

//2d field for user view
private Field2d fieldMaker = new Field2d();

  public SwerveSubsys() {
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      
      AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdomPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive1(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(0.05, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.05, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    
  }

  @Override
  public void periodic() {
    // updates odometry
    odometry.update(getHeading(), new SwerveModulePosition[] {
        frontLeftMod.getModPos(),
        frontRightMod.getModPos(),
        backLeftMod.getModPos(),
        backRightMod.getModPos()
    });

    //gets info on drive and steer Motor
    frontLeftMod.Info();
    frontRightMod.Info();
    backLeftMod.Info();
    backRightMod.Info();

    SmartDashboard.putNumber("Pigeon YAW", pigeon.getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Pigeon ANGLE", pigeon.getAngle());

// puts data into board to view
    fieldMaker.setRobotPose(getPose());
    SmartDashboard.putData(fieldMaker);
  }



  public Rotation2d getHeading(){
    //Returns the gyro heading might need invert if it is not clockwise positive
    return Rotation2d.fromDegrees(pigeon.getAngle());
    // return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble()); // add negative on pigeon angle
  }
//Might be used to see the current yaw and how far it is from its initial starting direction
//to autocorrect itself in the right orientation
  public double getYaw(){
    return pigeon.getAngle();
  }

  public void resetHeading(){
    //resets Gyro heading on field
    //Change to .setYaw(0)???
    //reset();
    System.out.println("HEADING IS RESETING");
    pigeon.reset();
    //This is the 0 of the robot
    pigeon.setYaw(0);
  }

  public Pose2d getPose(){
    //returns the odometry pose or where we currently are after moving a ton
    return new Pose2d(
        new Translation2d(-odometry.getPoseMeters().getX(), -odometry.getPoseMeters().getY()),
        odometry.getPoseMeters().getRotation()
    );
  }



  public void resetPose(){
    //resets the odometry pose
    // for field references?
    //Utilizes the other Reset pose 
    resetOdomPose(new Pose2d());
    System.out.println("POSE IS RESETING");
  }

  //resets the orientation the robot moves in (I do not recommend using)
  public void resetOdomPose(Pose2d pose){
    //
    odometry.resetPosition(
        getHeading(),
        new SwerveModulePosition[] {
            frontLeftMod.getModPos(),
            frontRightMod.getModPos(),
            backLeftMod.getModPos(),
            backRightMod.getModPos()
        }, 
        pose);

  }

  //Gets Mods states 
  private ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.kinematics.toChassisSpeeds(frontLeftMod.getModState(),frontRightMod.getModState(),backLeftMod.getModState(),backRightMod.getModState());
  }

  //Drive that converts the speeds into robot orientation
  private void drive1(ChassisSpeeds chassisSpeeds){
    drive3(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false);
  }

  //if not using drive1 to convert it, then make the field relative false
  public void drive2(double xSpeed, double ySpeed, double rotSpeed){
    drive3(xSpeed,ySpeed,rotSpeed,false);
  }

  //This drive uses field orientation and needs to be changed since it is false there is no movement
  public void drive3(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative){
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    if(fieldRelative){
        //add negative on getHeading if need to invert
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, Rotation2d.fromRadians(getHeading().getRadians()));
    }

    SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(speeds);

    setModStates(states);
    
  }

  
  public void setModStates(SwerveModuleState[] states){
    //Reduce Speeds to attainable values
    //Add MaxVelocity by doing math if needed and place into attainable Max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Max_velo);
    
    //sets module states, MAKE SURE THEY ARE IN ORDER CALLED
    // In this case it was needed to invert them
    frontLeftMod.setModState(states[0]);
    frontRightMod.setModState(states[1]);
    backLeftMod.setModState(states[2]);
    backRightMod.setModState(states[3]);
  }




  public void stopMods(){
    //stops the modules 
    frontLeftMod.stop();
    frontRightMod.stop();
    backLeftMod.stop();
    backRightMod.stop();
  }





  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}