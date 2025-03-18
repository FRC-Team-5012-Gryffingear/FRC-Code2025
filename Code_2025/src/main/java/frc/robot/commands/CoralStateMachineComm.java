// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.limelightSubsystem;
import frc.robot.subsystems.intakeCombined;
import frc.robot.subsystems.ElevatorSubsys;

import frc.robot.commands.AprilTagAlignment;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CoralStateMachineComm extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final SwerveSubsys swerve;
    private final PoseEstimatorSubsystem poseEstimator;
    private final limelightSubsystem limelight;
    private final intakeCombined intake;
    private final ElevatorSubsys elevator;

    private static final double scoring_X_Offset = 0.5;
    private static final double collection_X_Offset = 1;
    private int coralsScored = 0;

    private enum State{
        INIT,
        GO_TO_SCORE1,
        SCORE1,
        GO_TO_COLLECTION1,
        COLLECT1,
        GO_TO_SCORE2,
        SCORE2,
        GO_TO_COLLECTION2,
        COLLECT2,
        GO_TO_SCORE3,
        SCORE3,
        GO_TO_COLLECTION3,
        COLLECT3,
        GO_TO_SCORE4,
        SCORE4,
        FINISHED
    }

    private State currentState = State.INIT;
    private Command currentCommand = null;
    private final Timer stateTimer = new Timer();

  /**
   * Creates a new CoralStateMachineComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CoralStateMachineComm(SwerveSubsys swervesubsys, PoseEstimatorSubsystem estimatorSubsys, limelightSubsystem limelightSubsys, intakeCombined intakeSubsys, ElevatorSubsys elevatorSubsys) {
    swerve = swervesubsys;
    poseEstimator = estimatorSubsys;
    limelight = limelightSubsys;
    intake = intakeSubsys;
    elevator = elevatorSubsys;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swervesubsys, intakeSubsys, elevatorSubsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.INIT;
    stateTimer.reset();
    stateTimer.start();
    coralsScored = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentState) {
        case INIT:
            currentState = State.GO_TO_SCORE1;
            stateTimer.reset();
            break;
        
        case GO_TO_SCORE1:

        default:
            break;
    }
  }

  private void handleNavigationToCollection(){
    if (currentCommand == null || currentCommand.isFinished()){
        currentCommand = new AprilTagAlignment(swerve, intake, elevator, limelight, scoring_X_Offset);
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
