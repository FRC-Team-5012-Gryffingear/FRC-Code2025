// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.testCom;
import frc.robot.commands.testCom2;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.test;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final test t = new test();
  private boolean toggle = true;
  // private final testCom tCom = new testCom(t, 0);
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  }

  public void setTogggle(){
    toggle = !toggle;
  }

  


  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    testCom first = new testCom(t, m_driverController);
    testCom2 second = new testCom2(t, -1);

    // t.setDefaultCommand(first);
    t.setDefaultCommand(new testCom(t, m_driverController));
    // t.setDefaultCommand();

    


    SmartDashboard.putBoolean("boolean", toggle);
    


    // m_driverController.a().toggleOnTrue(second);
    // m_driverController.a().toggleOnTrue(toggle ? new testCom(t, 1) : new testCom(t, -1));

    // Trigger te = m_driverController.a().toggleOnTrue(new InstantCommand(() -> tCom.setTogggle()));
    // System.out.println(te.getAsBoolean());

    // Trigger t2 = m_driverController.a().toggleOnFalse(new testCom(t, -1));
    
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
    // Autos.exampleAuto(m_exampleSubsystem);
  }
}
