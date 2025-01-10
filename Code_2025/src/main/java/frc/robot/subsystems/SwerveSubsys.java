// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveSubsys extends SubsystemBase {
  /** Creates a new SwerveSubsys. */
  private final SwerveMod backRightModule = new SwerveMod(
    Constants.BackRightDriveID, 
    Constants.BackRightSteerID, 
    Constants.BackRightEncoderID, 
    Constants.BackRightOffset, 
    false, 
    "Back Right Module");
  private final SwerveMod backLeftModule = new SwerveMod(
    Constants.BackLeftDriveID, 
    Constants.BackLeftSteerID, 
    Constants.BackLeftEncoderID, 
    Constants.BackLeftOffset, 
    false, 
    "Back Left Module");
  private final SwerveMod frontRightModule = new SwerveMod(
     Constants.FrontRightDriveID,
     Constants.FrontRightSteerID, 
     Constants.FrontRightEncoderID, 
     Constants.FrontRightOffset, 
     false, 
     "Front Right Module");
  private final SwerveMod frontLeftModule = new SwerveMod(
    Constants.FrontLeftDriveID,
    Constants.FrontLeftSteerID,
    Constants.FrontLeftEncoderID,
    Constants.FrontLeftOffset,
    false,
    "Front Left Module");

  private final Pigeon2 pigeon = new Pigeon2(Constants.PigeonID);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.kinematics, getHeading(), new SwerveModulePosition[]{
    frontLeftModule.getModPos(),
    frontRightModule.getModPos(),
    backLeftModule.getModPos(),
    backRightModule.getModPos()
  });

  private Field2d fieldMaker = new Field2d();

  public SwerveSubsys() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(),
     new SwerveModulePosition[] {
        frontLeftModule.getModPos(),
        frontRightModule.getModPos(),
        backLeftModule.getModPos(),
        backRightModule.getModPos()
     });

    frontLeftModule.Info();
    frontRightModule.Info();
    backLeftModule.Info();
    backRightModule.Info();

    fieldMaker.setRobotPose(getPose());
    SmartDashboard.putData(fieldMaker);
  }

  public Rotation2d getHeading(){
    //Returns the gyro heading might need invert if it is not clockwise positive
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble()); // add negative if angle not right
  }

  // If we want to implement the auto correction yaw 
  // Will need changes due to software changes
  public double getYaw(){
    double currentYaw = pigeon.getYaw().getValueAsDouble();
    double angle = currentYaw % 360;
    return angle;
  }

  public void resetHeading(){
    // This resets the gyro heading on the field
    // Might need to adjust the Yaw default
    System.out.println("HEADING IS RESETING");
    pigeon.reset();
    pigeon.setYaw(0); // Adjust according to the "Forward of the bot"
  }

  public Pose2d getPose(){
    // returns the odometry pose or where we currently are
    return new Pose2d(
        new Translation2d(-odometry.getPoseMeters().getX(), -odometry.getPoseMeters().getY()),
        odometry.getPoseMeters().getRotation()
    );
  }

  public void resetPose(){
    // resets the odometry pose
    // Utilizes the reset odom function
    resetOdomPose(new Pose2d());
    System.out.println("Pose is reseting");
  }

  // Resets the orientation of the robot, I do not reccomend using unless it is for auto trajectory
  public void resetOdomPose(Pose2d pose){
    odometry.resetPosition(getHeading(), 
    new SwerveModulePosition[] {
        frontLeftModule.getModPos(),
        frontRightModule.getModPos(),
        backLeftModule.getModPos(),
        backRightModule.getModPos()
    }, 
    pose);
  }

  // Gets Mods speeds for reference(dx, dy, theta)
  private ChassisSpeeds getRobotrelativeSpeeds(){
    return Constants.kinematics.toChassisSpeeds(frontLeftModule.getModState(),frontRightModule.getModState(),backLeftModule.getModState(),backRightModule.getModState());
  }

  // This is Swerve on robot orientation rather than field orientation
  private void drive1(ChassisSpeeds chassisSpeeds){
    drive3(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false);
  }

  // This one utilizes Drive3 function as field orientation
  public void drive2(double xSpeed, double ySpeed, double rotSpeed){
    drive3(xSpeed,ySpeed,rotSpeed,true);
  }

  // This funciton serves as a blueprint for the other two functions
  // However it is prefered to use only this one as the main field orientation
  // and have the drive2 as the robot-orientation due to ChassisSpeeds requirement on the first one
  public void drive3(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative){
    ChassisSpeeds Speeds = new ChassisSpeeds(xSpeed,ySpeed,rotSpeed);
    if(fieldRelative){
        Speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed,rotSpeed,Rotation2d.fromDegrees(getHeading().getDegrees()));
    }
    
    SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(Speeds);
    
    setModStates(states);
  }

  public void setModStates(SwerveModuleState[] states){

    // This makes sures that the speeds do not exceed specific value
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Max_velo);

    //Sets the modules states. THE ORDER MAY VARY CHANGE ACCORDINGLY
    frontLeftModule.setModState(states[0]);
    frontRightModule.setModState(states[1]);
    backLeftModule.setModState(states[2]);
    backRightModule.setModState(states[3]);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
