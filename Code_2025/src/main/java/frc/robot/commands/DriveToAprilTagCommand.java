package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionMemorySubsystem;
import java.util.Optional;

/**
 * DriveToAprilTagCommand - Navigate to April tag using MegaTag2
 * 
 * Uses current robot position (from MegaTag2 or memory) and target April tag
 * position (from field layout) to drive toward the tag dynamically.
 * 
 * No pre-planned paths - entirely position-based navigation.
 */
public class DriveToAprilTagCommand extends Command {
    
    private final SwerveSubsystem swerve;
    private final LimelightSubsystem limelight;
    private final VisionMemorySubsystem visionMemory;
    
    private final int targetAprilTagID;
    private final double approachDistance;  // How close to get to tag (meters)
    
    // PID Controllers for each axis
    private final PIDController xPID = new PIDController(0.5, 0.0, 0.02);
    private final PIDController yPID = new PIDController(0.5, 0.0, 0.02);
    private final PIDController rotationPID = new PIDController(0.05, 0.0, 0.01);
    
    // Speed limits
    private static final double MAX_LINEAR_SPEED = 2.0;   // m/s
    private static final double MAX_ANGULAR_SPEED = 2.0;  // rad/s
    
    // Position tolerance for completion
    private static final double POSITION_TOLERANCE = 0.05;  // 5cm
    private static final double ROTATION_TOLERANCE = 5.0;   // 5 degrees
    
    public DriveToAprilTagCommand(
            SwerveSubsystem swerve,
            LimelightSubsystem limelight,
            VisionMemorySubsystem visionMemory,
            int targetAprilTagID,
            double approachDistance) {
        
        this.swerve = swerve;
        this.limelight = limelight;
        this.visionMemory = visionMemory;
        this.targetAprilTagID = targetAprilTagID;
        this.approachDistance = approachDistance;
        
        // Require swerve subsystem
        addRequirements(swerve);
        
        // Set PID tolerances
        xPID.setTolerance(POSITION_TOLERANCE);
        yPID.setTolerance(POSITION_TOLERANCE);
        rotationPID.setTolerance(Math.toRadians(ROTATION_TOLERANCE));
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putString("Command Status", "Initializing drive to tag " + targetAprilTagID);
    }
    
    @Override
    public void execute() {
        // Get current robot position
        Optional<Pose2d> robotPoseOpt = getCurrentRobotPosition();
        if (robotPoseOpt.isEmpty()) {
            // No position available, stop driving
            stopDriving();
            SmartDashboard.putString("Command Status", "NO POSITION AVAILABLE");
            return;
        }
        
        Pose2d robotPose = robotPoseOpt.get();
        
        // Get target April tag position
        Optional<Pose2d> tagPoseOpt = limelight.getAprilTagPose(targetAprilTagID);
        if (tagPoseOpt.isEmpty()) {
            // Invalid tag ID
            stopDriving();
            SmartDashboard.putString("Command Status", "INVALID TAG ID");
            return;
        }
        
        Pose2d tagPose = tagPoseOpt.get();
        
        // Calculate approach point (back away from tag by approachDistance)
        Pose2d approachPose = calculateApproachPose(tagPose);
        
        // Calculate error
        double errorX = approachPose.getX() - robotPose.getX();
        double errorY = approachPose.getY() - robotPose.getY();
        double errorRotation = normalizeAngle(
            approachPose.getRotation().getDegrees() - robotPose.getRotation().getDegrees()
        );
        
        // PID calculations
        double vx = xPID.calculate(0, errorX);
        double vy = yPID.calculate(0, errorY);
        double omega = rotationPID.calculate(0, Math.toRadians(errorRotation));
        
        // Limit speeds
        vx = Math.max(-MAX_LINEAR_SPEED, Math.min(MAX_LINEAR_SPEED, vx));
        vy = Math.max(-MAX_LINEAR_SPEED, Math.min(MAX_LINEAR_SPEED, vy));
        omega = Math.max(-MAX_ANGULAR_SPEED, Math.min(MAX_ANGULAR_SPEED, omega));
        
        // Drive robot (field-oriented)
        swerve.getSwerveDrive().drive(
            new ChassisSpeeds(vx, vy, omega),
            true,  // Field-oriented
            new Translation2d()
        );
        
        // Publish telemetry
        publishTelemetry(robotPose, tagPose, errorX, errorY, errorRotation);
    }
    
    @Override
    public void end(boolean interrupted) {
        stopDriving();
        SmartDashboard.putString("Command Status", interrupted ? "INTERRUPTED" : "COMPLETE");
    }
    
    @Override
    public boolean isFinished() {
        
        // Check if we're close enough to target
        Optional<Pose2d> robotPoseOpt = getCurrentRobotPosition();
        Optional<Pose2d> tagPoseOpt = limelight.getAprilTagPose(targetAprilTagID);
        
        if (robotPoseOpt.isEmpty() || tagPoseOpt.isEmpty()) {
            return false;
        }
        
        Pose2d robotPose = robotPoseOpt.get();
        Pose2d approachPose = calculateApproachPose(tagPoseOpt.get());
        
        // Check position error
        double positionError = robotPose.getTranslation()
            .getDistance(approachPose.getTranslation());
        
        // Check rotation error
        double rotationError = Math.abs(
            normalizeAngle(
                approachPose.getRotation().getDegrees() - 
                robotPose.getRotation().getDegrees()
            )
        );
        
        return positionError < POSITION_TOLERANCE && 
               rotationError < ROTATION_TOLERANCE;
    }
    
    /**
     * Get current robot position from MegaTag2 or memory
     * Priority: MegaTag2 (fresh) > Memory (recent) > Empty
     */
    private Optional<Pose2d> getCurrentRobotPosition() {
        // Try MegaTag2 first (if fresh)
        if (limelight.isMegaTagDataFresh()) {
            return Optional.of(limelight.getRobotPose());
        }
        
        // Fall back to memory
        if (visionMemory.hasValidMemory()) {
            return visionMemory.getStoredRobotPose();
        }
        
        // No position available
        return Optional.empty();
    }
    
    /**
     * Calculate ideal approach pose
     * Back away from tag by approachDistance
     */
    private Pose2d calculateApproachPose(Pose2d tagPose) {
        // Simple approach: position directly in front of tag at approachDistance
        // Heading: face toward the tag
        double distance = approachDistance;
        
        // Move back from tag along direction we're facing
        double approachX = tagPose.getX() - distance;
        double approachY = tagPose.getY();
        
        return new Pose2d(
            approachX,
            approachY,
            tagPose.getRotation()  // Face same direction as tag
        );
    }
    
    /**
     * Normalize angle to -180 to +180 degrees
     */
    private double normalizeAngle(double degrees) {
        double normalized = degrees % 360;
        if (normalized > 180) {
            normalized -= 360;
        } else if (normalized < -180) {
            normalized += 360;
        }
        return normalized;
    }
    
    /**
     * Stop driving
     */
    private void stopDriving() {
        swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0), true, new Translation2d());
    }
    
    /**
     * Publish debugging telemetry
     */
    private void publishTelemetry(
            Pose2d robotPose,
            Pose2d tagPose,
            double errorX,
            double errorY,
            double errorRotation) {
        
        SmartDashboard.putString("Command Status", "DRIVING");
        SmartDashboard.putNumber("Target Tag ID", targetAprilTagID);
        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Tag X", tagPose.getX());
        SmartDashboard.putNumber("Tag Y", tagPose.getY());
        SmartDashboard.putNumber("Error X (m)", errorX);
        SmartDashboard.putNumber("Error Y (m)", errorY);
        SmartDashboard.putNumber("Error Rotation (deg)", errorRotation);
    }
}
