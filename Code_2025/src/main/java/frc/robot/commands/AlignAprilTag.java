// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.limeyImproved;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlignAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsys swerve;
  private final limeyImproved limelight;
  //Change PID values to tune the PID loop
  private final PIDController xPID = new PIDController(0.0, 0.0, 0.0);
  private final PIDController yPID = new PIDController(0.0, 0.0, 0.0);
  private final PIDController rotPID = new PIDController(0.0, 0.0, 0.0);
  private boolean initialized = false;
  private double x2, y2, rot2;
  /**
   * Creates a new AlignAprilTag.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignAprilTag(SwerveSubsys subsystem, limeyImproved lime) {
    swerve = subsystem;
    limelight = lime;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Alternative: RawFiducial detectedID = limelight.getFiducial(X); where X is the ID of the apriltag
    //Everything could be placed in a function onwards.
    RawFiducial detectedID = limelight.getFiducial();
    if (!(detectedID == null)) {


        //Alternative way of getting the x, y, and rot values from the limelight but w/o while loop
        if(!initialized){
            x2 = detectedID.txnc;
            y2 = detectedID.distToRobot;
            rot2 = Math.atan(y2 / x2); //May need tweaking to get the right value
        }
        initialized = true;
        //May need to cap values?
        double xOutput2 = xPID.calculate(x2, 0);
        double yOutput2 = yPID.calculate(y2, 15);
        double rotOutput2 = rotPID.calculate(rot2, 0);
        swerve.drive3(xOutput2, yOutput2, rotOutput2, false); //Change field relative if necessary.
        x2 -= xOutput2;
        y2 -= yOutput2;
        rot2 -= rotOutput2;
        //IMPORTANT NOTE, may need to set setpoints earlier?
        if(xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint()) {
            swerve.drive3(0, 0, 0, false); //Change field relative if necessary.
            initialized = false; //This will cause errors, discuss
          }

        SmartDashboard.putNumber("X", x2);
        SmartDashboard.putNumber("Y", y2);
        SmartDashboard.putNumber("Rot", rot2);
        SmartDashboard.putNumber("X Output", xOutput2);
        SmartDashboard.putNumber("Y Output", yOutput2);
        SmartDashboard.putNumber("Rot Output", rotOutput2);

        
        
        
        
        
        
        
        //Get the x, y, and rot values from the limelight using static variables.
        double x1 = detectedID.txnc;
        double y1 = detectedID.distToRobot;
        double rot1 = Math.atan(y1 / x1); //May need tweaking to get the right value

        while(!(xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint())){
            //May need to cap values?
            double xOutput1 = xPID.calculate(x1, 0);
            double yOutput1 = yPID.calculate(y1, 15);
            double rotOutput1 = rotPID.calculate(rot1, 0);
            swerve.drive3(xOutput1, yOutput1, rotOutput1, false); //Change field relative if necessary.
            //Hypothetical way of decreasing x1, y1, rot1 to decrease themselves.
            x1 -= xOutput1;
            y1 -= yOutput1;
            rot1 -= rotOutput1;
            SmartDashboard.putNumber("X", x1);
            SmartDashboard.putNumber("Y", y1);
            SmartDashboard.putNumber("Rot", rot1);
            SmartDashboard.putNumber("X Output", xOutput1);
            SmartDashboard.putNumber("Y Output", yOutput1);
            SmartDashboard.putNumber("Rot Output", rotOutput1);
        }







        //Get the x, y, and rot values from the limelight with continous movement
        double x = detectedID.txnc;
        double y = detectedID.distToRobot;
        double rot = Math.atan(y / x); //May need tweaking to get the right value
        double xOutput = xPID.calculate(x, 0);
        double yOutput = yPID.calculate(y, 15);
        double rotOutput = rotPID.calculate(rot, 0);
        swerve.drive3(x, y, rotOutput, false);
        if(xPID.atSetpoint() && yPID.atSetpoint() && rotPID.atSetpoint()) {
            swerve.drive3(0, 0, 0, false); //Change field relative if necessary.
          }
        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("Rot", rot);
        SmartDashboard.putNumber("X Output", xOutput);
        SmartDashboard.putNumber("Y Output", yOutput);
        SmartDashboard.putNumber("Rot Output", rotOutput);

    } else {
        swerve.drive3(0, 0, 15, false);
    }
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
