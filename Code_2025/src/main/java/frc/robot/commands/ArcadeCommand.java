// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArcadeSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.MsvcRuntimeException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArcadeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArcadeSubsystem m_subsystem;
  private final DoubleSupplier RT, LT, TT;
  private final PIDController rotPID = new PIDController(0.1, 0, 0);
  private double storeYawValue = 0;
  /*BooleanSupplier here
   * and add it to the system/function
   */
  private final BooleanSupplier AT, CorrectionButton;
  private boolean holdingDown = true;

  /**
   * Creates a new ArcadeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeCommand(ArcadeSubsystem subsystem, DoubleSupplier R, DoubleSupplier L, DoubleSupplier T,BooleanSupplier A, BooleanSupplier C) {
    m_subsystem = subsystem;
    RT = R;
    LT = L;
    TT = T;
    AT = A;
    CorrectionButton = C;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetYaw();
    storeYawValue = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Volt", m_subsystem.getVoltage());
    //If the correction button is pressed and it has not been held down yet, then it is set as held down / resetting the yaw to 0.

    // if(Math.abs(TT.getAsDouble()) < 0.1){
    //   PIDController control = new PIDController(0.01, 0, 0);
    //   double speed = control.calculate(m_subsystem.getYaw(), storeYawValue);
    //   m_subsystem.moveAndTurn(RT.getAsDouble()-LT.getAsDouble(), speed);

    // }else{
    //   m_subsystem.moveAndTurn(RT.getAsDouble()-LT.getAsDouble(), TT.getAsDouble());

    //   storeYawValue = m_subsystem.getYaw();
    // }
    m_subsystem.moveAndTurn(RT.getAsDouble()-LT.getAsDouble(), TT.getAsDouble());

    m_subsystem.coralouttake(AT.getAsBoolean());
    SmartDashboard.putNumber("Stored yaw", storeYawValue);
    SmartDashboard.putNumber("Current yaw", m_subsystem.getYaw());
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
