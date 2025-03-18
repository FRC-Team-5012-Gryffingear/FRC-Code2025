// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.intakePneumatics;
// import frc.robot.subsystems.intakeCombined;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Autos extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveSubsys m_subsystem;
  private Timer time = new Timer();
  // private final ElevatorSubsys elev = new ElevatorSubsys();
  // // private final intakeCombined intake = new intakeCombined();
  // private final intakePneumatics intake = new intakePneumatics();

  
  public Autos(SwerveSubsys subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.stop();
    time.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time.start();
    if(time.get() > 4 && time.get() < 6){
      m_subsystem.drive3(-1, 0, 0, true);
    }
    else if(time.get() > 6){
      m_subsystem.drive3(0, 0, 0, true);
    }
    // if(time.get()>0){
    //   if(elev.getEncoderPos() <= 9.8){
    //     elev.elevMovement(9.8);
    //   }else{
    //     elev.elevUpAndDown(0);
    //     intake.reverseHook();
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    time.stop();
    return false;
  }
}
