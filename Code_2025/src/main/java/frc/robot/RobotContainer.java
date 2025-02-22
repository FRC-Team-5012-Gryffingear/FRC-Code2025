// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignAprilTag;
import frc.robot.commands.ElevatorCom;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeUp;
import frc.robot.commands.SwerveCom;
import frc.robot.commands.intakeCoral;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsysCoral;
import frc.robot.subsystems.IntakeSubsysLift;
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
  private final IntakeSubsysCoral intakeCor = new IntakeSubsysCoral();
  private final IntakeSubsysLift intakeLift = new IntakeSubsysLift();

  private final ElevatorSubsys elev = new ElevatorSubsys();

  private final AlignAprilTag tagMove = new AlignAprilTag(swerve, limeI,elev,intakeCor,intakeLift);



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
      elev,
      () -> driverController.a().getAsBoolean()));

      //Testing uses only, temporary
      elev.setDefaultCommand(new ElevatorCom(elev,
      operatorController,
      0));


    configureBindings();
  }

 
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .toggleOnTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // Command open_Intake = new intakeCoral(intakeCor, 1);
    // Command close_Intake = new intakeCoral(intakeCor, -1);

    // intakeCor.setDefaultCommand(close_Intake);
    // operatorController.leftBumper().toggleOnTrue(open_Intake);


    // Command lift_Intake = new IntakeUp(intakeLift,1);
    // Command lower_intake = new IntakeUp(intakeLift, -1);

    // intakeLift.setDefaultCommand(lower_intake);
    // operatorController.rightBumper().toggleOnTrue(lift_Intake);


    // operatorController.leftStick().toggleOnTrue(new ElevatorCom(elev, 0.75)); //1st level 
    // operatorController.a().toggleOnTrue(new ElevatorCom(elev, 2.41)); // 2st level
    // operatorController.b().toggleOnTrue(new ElevatorCom(elev, 5.2)); // 3nd level
    // operatorController.x().toggleOnTrue(new ElevatorCom(elev, 9.65)); // 4rd level
    // operatorController.y().toggleOnTrue(new ElevatorCom(elev, 1.2)); // Human player station
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