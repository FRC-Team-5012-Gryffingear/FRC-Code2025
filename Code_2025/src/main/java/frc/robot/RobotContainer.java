// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignAprilTag;
import frc.robot.commands.ElevatorCom;
import frc.robot.commands.ExampleCommand;
// import frc.robot.commands.IntakeUp;
import frc.robot.commands.SwerveCom;
// import frc.robot.commands.intakeCoral;
import frc.robot.commands.intakeState1;
import frc.robot.commands.intakeState2;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
// import frc.robot.subsystems.IntakeSubsysCoral;
// import frc.robot.subsystems.IntakeSubsysLift;
import frc.robot.subsystems.SwerveSubsys;
import frc.robot.subsystems.intakePneumatics;
import frc.robot.subsystems.limelightSubsystem;
import frc.robot.subsystems.limeyImproved;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final limey lime = new limey();

  private final limeyImproved limeI = new limeyImproved();
  private final limelightSubsystem lime = new limelightSubsystem();
  private final SwerveSubsys swerve = new SwerveSubsys();
  // private final IntakeSubsysCoral intakeCor = new IntakeSubsysCoral();
  // private final IntakeSubsysLift intakeLift = new IntakeSubsysLift();
  private final intakePneumatics combined = new intakePneumatics();

  private final ElevatorSubsys elev = new ElevatorSubsys();

  private final PoseEstimatorSubsystem poseEst = new PoseEstimatorSubsystem(swerve, lime);

  private final AlignAprilTag tagMove = new AlignAprilTag(swerve, limeI,elev, combined);

  private final SendableChooser<Command> autoChooser;


  // private final limeyImproved limeI = new limeyImproved();
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DriverContrlPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OperatorContrlPort);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    // lime.setDefaultCommand(new limeyCom(lime));

    // SmartDashboard.putString("Senior view", "limelight-senior/stream.mjpg");
    // limeI.setDefaultCommand(new AlignAprilTag(swerve, limeI));
     AutoBuilderconfigure();
     autoChooser = AutoBuilder.buildAutoChooser();

     swerve.setDefaultCommand(new SwerveCom(
      swerve, 
      driverController,
      elev,
      () -> driverController.a().getAsBoolean()));

      //Testing uses only, temporary
    
      elev.setDefaultCommand(new ElevatorCom(elev,operatorController));


    configureBindings();
    autoChooserConfig();
  }

 
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .toggleOnTrue(new ExampleCommand(m_exampleSubsystem));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    operatorController.leftBumper().onTrue(new intakeState1(combined));
    operatorController.rightBumper().onTrue(new intakeState2(combined));


    // double pow = 1;
    // Command hook_move1 = new IntakeUp(intakeLift, 1);
    // Command hook_move2 = new IntakeUp(intakeLift, -1);

    // intakeLift.setDefaultCommand(hook_move1);
    // operatorController.rightBumper().toggleOnTrue(hook_move2);



    // Command open_Intake = new intakeCoral(intakeCor, pow);
    // Command close_Intake = new intakeCoral(intakeCor, -pow);

    // intakeCor.setDefaultCommand(close_Intake);
    // operatorController.leftBumper().toggleOnTrue(open_Intake);


    
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


  private void autoChooserConfig(){
    autoChooser.addOption("Option 1", new PathPlannerAuto("Auto Path"));
    SmartDashboard.putData("AutoChooser", autoChooser);

  }
  
  private void AutoBuilderconfigure(){
    RobotConfig config;

    try{
      config = RobotConfig.fromGUISettings();
    
      // Configure AutoBuilder last
      AutoBuilder.configure(
            poseEst::getCurrentPose, // Robot pose supplier
            poseEst::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> swerve.drive1(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            swerve // Reference to this subsystem to set requirements
        );

      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      };
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   
      return new PathPlannerAuto("Test Auto");// Autos.exampleAuto(m_exampleSubsystem);
  
  }
}