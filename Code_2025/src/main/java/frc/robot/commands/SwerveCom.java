// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.limey;

/** An example command that uses an example subsystem. */
public class SwerveCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsys swerve;
 private final CommandXboxController controller2;
  // private final BooleanSupplier yaw;

  // private final BooleanSupplier limeLock;

  //private final LedSubsystem Ledsubsys; 
  public double t = 1;


 
  
  public SwerveCom(SwerveSubsys subsystem, CommandXboxController controller) {
    swerve = subsystem;
    controller2 = controller;
    
  
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //vision.startThread();
    swerve.resetHeading();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = MathUtil.applyDeadband(controller2.getLeftY(), OperatorConstants.Deadband) * 0.4;
    double ySpeed = MathUtil.applyDeadband(controller2.getLeftX(), OperatorConstants.Deadband) * 0.4;
    double rotateSpeed = MathUtil.applyDeadband(controller2.getRightX(), OperatorConstants.Deadband) * 0.4;
    
    
    swerve.drive3(xSpeed, -ySpeed, rotateSpeed*1.5, true);

    

    

     // Function that moves towards the target certain amount



    //Function that moves left/right in order to look at target
   
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopMods();
   // vision.stopThread();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}