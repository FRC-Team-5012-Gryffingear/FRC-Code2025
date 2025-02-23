// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.IntakeSubsysCoral;
import frc.robot.subsystems.IntakeSubsysLift;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class IntakeUp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsysLift intake;
  private double pow;
  private final CommandXboxController operator = new CommandXboxController(OperatorConstants.OperatorContrlPort);
  private boolean toggle = false;

  /**
   * Creates a new IntakeElevCom.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeUp(IntakeSubsysLift subsystem, double pow) {
    intake = subsystem;
    this.pow = pow;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // lift and lower intake angle
    intake.up(pow);
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
