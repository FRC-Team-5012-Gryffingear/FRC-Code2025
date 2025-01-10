// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.limey;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class limeyCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final limey lime;

  /**
   * Creates a new limeyCom.
   *
   * @param subsystem The subsystem used by this command.
   */
  public limeyCom(limey subsystem) {
    lime = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Rot Fwd Lock Val", lime.rotAround(lime.getX()));
    SmartDashboard.putNumber("Rot turn Lock val", lime.rotationLock(lime.getX()));
    SmartDashboard.putNumber("Fwd Lock val", lime.fwrdLock(lime.estimate3DZInches()));
    System.out.println("WORKING");
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
