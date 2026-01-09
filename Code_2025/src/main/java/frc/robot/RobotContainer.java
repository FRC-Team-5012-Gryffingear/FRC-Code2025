// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToReefCommand;
import frc.robot.commands.ElevatorCom;
import frc.robot.commands.intakeState1;
import frc.robot.commands.intakeState2;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwervePoseEstimatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intakePneumatics;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final intakePneumatics intakePneu = new intakePneumatics();

  private final ElevatorSubsys elev = new ElevatorSubsys(intakePneu);
  private final LimelightSubsystem limelight = new LimelightSubsystem();
    private final SwervePoseEstimatorSubsystem poseEstimator = 
        new SwervePoseEstimatorSubsystem(drivebase, limelight);



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DriverContrlPort);
  private final CommandXboxController operatorController = 
      new CommandXboxController(OperatorConstants.OperatorContrlPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1.8)
                                                            .allianceRelativeControl(true);

 SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().
 withControllerHeadingAxis(driverXbox::getRightX, driverXbox::getRightY)
 .headingWhile(true);



  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward());

    autoChooser.addOption("New Weird Path", drivebase.DriveWithWayPoints());

    autoChooser.addOption("Auto for Test", new PathPlannerAuto("New Auto"));
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }


  

 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    Command driveFieldOrientedDirectAngle2 = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());
    
    
    driverXbox.a().onTrue(
    Commands.runOnce(() -> {
        Pose2d currentPose = drivebase.getSwerveDrive().getPose();
        Pose2d newPose = new Pose2d(
            currentPose.getTranslation(),
            Rotation2d.fromDegrees(0)
        );
        drivebase.getSwerveDrive().resetOdometry(newPose);
    })
);
     driverXbox.b().onTrue(
            new AlignToReefCommand(drivebase, poseEstimator, limelight, limelight.getTagID(), 0.4)
        );

    


    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);





    operatorController.leftBumper().onTrue(new intakeState1(intakePneu));
    operatorController.rightBumper().onTrue(new intakeState2(intakePneu));
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
   return new PathPlannerAuto("Auton test #1");

    // Autos.exampleAuto(m_exampleSubsystem);
  }
}
