// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.pneumatic;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class pneumaticCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final ExampleSubsystem m_subsystem;
    private final pneumatic pneu;
    private final BooleanSupplier a, b;

  /**
   * Creates a new pneumaticCom.
   *
   * @param subsystem The subsystem used by this command.
   */
  public pneumaticCom(pneumatic subsystem, BooleanSupplier A, BooleanSupplier B) {
    pneu = subsystem;
    a = A;
    b = B;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneu.pneumaticMove(a.getAsBoolean(), b.getAsBoolean());
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
