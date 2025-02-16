// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignAprilTag;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCom;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveCom;
import frc.robot.commands.intakeCom;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsys;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.limeyImproved;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final limey lime = new limey();

  private final limeyImproved limeI = new limeyImproved();
  private final SwerveSubsys swerve = new SwerveSubsys();
  private final IntakeSubsys intake = new IntakeSubsys();
  private final ElevatorSubsys elev = new ElevatorSubsys();

  private final AlignAprilTag tagMove = new AlignAprilTag(swerve, limeI);



  // private final limeyImproved limeI = new limeyImproved();
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DriverContrlPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OperatorContrlPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // lime.setDefaultCommand(new limeyCom(lime));

    // limeI.setDefaultCommand(new AlignAprilTag(swerve, limeI));

     swerve.setDefaultCommand(new SwerveCom(
      swerve, 
      driverController,
      () -> driverController.a().getAsBoolean()));

      //Testing uses only, temporary
      elev.setDefaultCommand(new ElevatorCom(elev,
      operatorController,
      0));

    //  intake.setDefaultCommand(new intakeCom(intake, 
    //  () -> operatorController.leftBumper().getAsBoolean(), 
    //  () -> operatorController.rightBumper().getAsBoolean()));

    configureBindings();
  }

 
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .toggleOnTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());


    
    // operatorController.leftStick().toggleOnTrue(new ElevatorCom(elev, 0)); //1st default position
    // operatorController.a().toggleOnTrue(new ElevatorCom(elev, 0)); // 2st level
    // operatorController.b().toggleOnTrue(new ElevatorCom(elev, 4.8)); // 3nd level
    // operatorController.x().toggleOnTrue(new ElevatorCom(elev, 0)); // 4rd level
    // operatorController.y().toggleOnTrue(new ElevatorCom(elev, 0)); // Human player station
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return tagMove;// Autos.exampleAuto(m_exampleSubsystem);
  }
}