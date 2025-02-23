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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
      elev.setDefaultCommand(new ElevatorCom(elev,operatorController));


    configureBindings();
  }

 
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .toggleOnTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    double pow = 1;
    Command hook_move1 = new IntakeUp(intakeLift, 1);
    Command hook_move2 = new IntakeUp(intakeLift, -1);

    intakeLift.setDefaultCommand(hook_move1);
    operatorController.rightBumper().toggleOnTrue(hook_move2);



    Command open_Intake = new intakeCoral(intakeCor, pow);
    Command close_Intake = new intakeCoral(intakeCor, -pow);

    intakeCor.setDefaultCommand(close_Intake);
    operatorController.leftBumper().toggleOnTrue(open_Intake);


    
    // Bad naming it controls individually the hooks
    
    // operatorController.rightBumper;

    // Command lift_Intake = new IntakeUp(intakeLift,-1);
    // Command lower_intake = new IntakeUp(intakeLift, 1);

    // intakeLift.setDefaultCommand(lower_intake);
    // operatorController.rightBumper().toggleOnTrue(lift_Intake);


    operatorController.y().whileTrue(new ElevatorCom(elev, operatorController)); //1st level 0.75
    operatorController.a().whileTrue(new ElevatorCom(elev,operatorController)); // 2st level 2.41
    operatorController.b().whileTrue(new ElevatorCom(elev,operatorController)); // 3nd level 5.2 
    operatorController.x().whileTrue(new ElevatorCom(elev, operatorController)); // 4rd level  9.65
    operatorController.leftStick().whileTrue(new ElevatorCom(elev, operatorController)); // Human player station  1.2
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