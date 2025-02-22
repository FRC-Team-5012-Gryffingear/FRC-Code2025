// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.test;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class testCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final test m_subsystem;
  private boolean toggle = true;
//   private final BooleanSupplier a,b;
  private double pow;

  private RobotContainer c;
  /**
   * Creates a new testCom.
   *
   * @param subsystem The subsystem used by this command.
   */
  public testCom(test subsystem, double pow) {
    m_subsystem = subsystem;
    this.pow = pow;
    // this.a = a;
    // this.b = b;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public void setTogggle(){
    toggle = !toggle;
  }

  public double setPower(){
    if(toggle){
      return 1;
    } else{
      return -1;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_subsystem.tester(false, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(a.getAsBoolean()){
    //     m_subsystem.tester(true, false);
    // }
    // else if(b.getAsBoolean()){
    //     m_subsystem.tester(false, true);
    // }
    m_subsystem.tester(pow);
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
