// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsys;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class SwerveCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsys m_subsystem;
  private final CommandXboxController controller;
  private final BooleanSupplier yaw; // ADD reset POSE IF NECESSARY

  /**
   * Creates a new SwerveCom.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SwerveCom(SwerveSubsys subsystem, CommandXboxController controller, BooleanSupplier yaw) {
    m_subsystem = subsystem;
    this.controller = controller;
    this.yaw = yaw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.Deadband) * 0.4;
    double ySpeed = MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.Deadband) * 0.4;
    double rotateSpeed = MathUtil.applyDeadband(controller.getRightX(), OperatorConstants.Deadband) * 0.4;

    m_subsystem.drive3(xSpeed, -ySpeed, rotateSpeed, true);

    if(yaw.getAsBoolean()){
        m_subsystem.resetHeading();
    }

    // ADD RESET POSE IF NECESSARY HERE
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopMods();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
