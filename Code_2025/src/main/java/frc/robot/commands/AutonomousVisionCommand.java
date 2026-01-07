package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionMemorySubsystem;

/**
 * AutonomousVisionCommand - Complete vision-guided autonomous routine
 * 
 * Sequence:
 * 1. Wait for initial localization
 * 2. Drive to target April tag
 * 3. Complete
 * 
 * No pre-planned paths - position-based navigation only.
 */
public class AutonomousVisionCommand extends SequentialCommandGroup {
    
    public AutonomousVisionCommand(
            SwerveSubsystem swerve,
            LimelightSubsystem limelight,
            VisionMemorySubsystem visionMemory,
            int targetAprilTagID,
            double approachDistance) {
        
        // Sequence:
        // 1. Give Limelight time to detect (if not already)
        addCommands(
            new WaitCommand(1.0)
        );
        
        // 2. Drive to April tag using vision
        addCommands(
            new DriveToAprilTagCommand(
                swerve,
                limelight,
                visionMemory,
                targetAprilTagID,
                approachDistance
            )
        );
        
        // 3. Small settling delay
        addCommands(
            new WaitCommand(0.5)
        );
        
        // Add your scoring commands here later:
        // addCommands(new ElevatorToHeightCommand(...));
        // addCommands(new IntakeScoreCommand(...));
    }
}
