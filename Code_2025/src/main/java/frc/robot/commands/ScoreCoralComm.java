// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intakePneumatics;
import frc.robot.subsystems.ElevatorSubsys;



/** An example command that uses an example subsystem. */
public class ScoreCoralComm extends ParallelCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ScoreCoralComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ScoreCoralComm(intakePneumatics intake, ElevatorSubsys elev, BooleanSupplier movementDone) {
    addCommands(
     new InstantCommand(() -> elev.elevMovement(9.38), elev),
     new SequentialCommandGroup(
       new WaitUntilCommand(()->elev.getEncoderPos() >= 9.38),
       new WaitUntilCommand(movementDone),
       new InstantCommand(() -> intake.reverseHook())       

     )


    );
  }

}

