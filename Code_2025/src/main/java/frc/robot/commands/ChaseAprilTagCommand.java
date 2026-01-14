package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LimelightPoseProvider;

/**
 * ChaseAprilTagCommand - Follows 7028 Jumpstart 2023/2024 pattern
 * 
 * Uses ProfiledPIDController for smooth, constrained movement
 * Follows the exact strategy from the videos:
 * 1. Get estimated pose
 * 2. Get tag pose from Limelight
 * 3. Calculate goal pose (offset from tag)
 * 4. Use PID to drive toward goal
 */
public class ChaseAprilTagCommand extends Command {
    
    private final SwerveSubsystem swerve;
    private final LimelightPoseProvider visionProvider;
    private final int targetTagId;
    private final double distanceFromTag;  // How far in front of tag to position
    
    // ProfiledPIDControllers (separate for each axis like in video)
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController rotController;
    
    // Constraints for smooth acceleration profiles
    private static final double MAX_VELOCITY_MPS = 2.0;
    private static final double MAX_ACCELERATION_MPS2 = 1.0;
    private static final double MAX_OMEGA_RPS = Math.PI;
    private static final double MAX_ALPHA_RPS2 = Math.PI;
    
    // PID gains (tune these!)
    private static final double KP_LINEAR = 2.0;
    private static final double KD_LINEAR = 0.06;
    private static final double KP_ANGULAR = 2.8;
    private static final double KD_ANGULAR = 0.23;
    
    // Tolerances
    private static final double TOLERANCE_LINEAR = 0.15;  // 5cm
    private static final double TOLERANCE_ANGULAR = edu.wpi.first.math.util.Units.degreesToRadians(4);  // 2°
    private static final double TIMEOUT_SECONDS = 5.0;
    
    private double startTime;
    
    /**
     * Chase toward an AprilTag
     * @param swerve Swerve subsystem
     * @param visionProvider Limelight pose provider
     * @param tagId Target tag ID to chase
     * @param distance Distance (meters) to maintain in front of tag
     */
    public ChaseAprilTagCommand(
        SwerveSubsystem swerve,
        LimelightPoseProvider visionProvider,
        int tagId,
        double distance
    ) {
        this.swerve = swerve;
        this.visionProvider = visionProvider;
        this.targetTagId = tagId;
        this.distanceFromTag = distance;
        
        // Create profiled PID controllers with constraints
        TrapezoidProfile.Constraints linearConstraints =
            new TrapezoidProfile.Constraints(MAX_VELOCITY_MPS, MAX_ACCELERATION_MPS2);
        TrapezoidProfile.Constraints angularConstraints =
            new TrapezoidProfile.Constraints(MAX_OMEGA_RPS, MAX_ALPHA_RPS2);
        
        xController = new ProfiledPIDController(KP_LINEAR, 0, KD_LINEAR, linearConstraints);
        yController = new ProfiledPIDController(KP_LINEAR, 0, KD_LINEAR, linearConstraints);
        rotController = new ProfiledPIDController(KP_ANGULAR, 0, KD_ANGULAR, angularConstraints);
        
        // Set tolerances (when to consider at goal)
        xController.setTolerance(TOLERANCE_LINEAR);
        yController.setTolerance(TOLERANCE_LINEAR);
        rotController.setTolerance(TOLERANCE_ANGULAR);
        
        // Rotation wraps around (can go -180° to +180°)
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(swerve, visionProvider);
    }
    
    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        System.out.println("🎯 Starting chase of tag " + targetTagId + 
                          " at distance " + distanceFromTag + "m");
    }
    
    @Override
    public void execute() {
        // CRITICAL: Update Limelight orientation every frame
        double gyroHeading = swerve.getSwerveDrive().getOdometryHeading().getDegrees();
        visionProvider.updateRobotOrientation(gyroHeading);
        
        // Get current pose from estimator (this is our starting point)
        Pose2d currentPose = visionProvider.getEstimatedPose();
        
        // Get vision measurement to find tag position
        var visionData = visionProvider.getRawVisionMeasurement();
        if (visionData == null || visionData.tagCount < 1) {
            // No tag visible - stop moving
            System.out.println("❌ No tags visible");
            swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
            return;
        }
        
        // Tag pose is already on field (from MegaTag2)
        Pose2d tagPose = visionData.pose;
        
        // Calculate goal: distance in front of tag, facing toward tag
        // "In front of tag" means negative X offset in tag's frame
        Pose2d goalPose = new Pose2d(
            tagPose.getX() - distanceFromTag * Math.cos(0),  // Assume facing 0°
            tagPose.getY() - distanceFromTag * Math.sin(0),
            new Rotation2d(0)  // Face tag (0° heading)
        );

    
        
        
        // Use ProfiledPIDControllers to calculate velocities (like in video)
        double vx = xController.calculate(visionProvider.getTopDownZ() + distanceFromTag,0);
        double vy = yController.calculate(visionProvider.getTopDownX(), 0);
        double omega = rotController.calculate(
            visionProvider.getTopDownYaw(), 0
        );
        
        // Clamp to max speeds
        vx = Math.max(-MAX_VELOCITY_MPS, Math.min(MAX_VELOCITY_MPS, vx));
        vy = Math.max(-MAX_VELOCITY_MPS, Math.min(MAX_VELOCITY_MPS, vy));
        omega = Math.max(-MAX_OMEGA_RPS, Math.min(MAX_OMEGA_RPS, omega));
        
        // Send to drivetrain
        swerve.getSwerveDrive().drive(new ChassisSpeeds(vx, -vy, -omega));
        
        // Telemetry
        SmartDashboard.putNumber("Chase/Current X", currentPose.getX());
        SmartDashboard.putNumber("Chase/Current Y", currentPose.getY());
        SmartDashboard.putNumber("Chase/Goal X", goalPose.getX());
        SmartDashboard.putNumber("Chase/Goal Y", goalPose.getY());
        SmartDashboard.putNumber("Chase/VX", vx);
        SmartDashboard.putNumber("Chase/VY", vy);
        SmartDashboard.putNumber("Chase/Omega", omega);
    }
    
    @Override
    public boolean isFinished() {
        double elapsed = Timer.getFPGATimestamp() - startTime;
        
        boolean atGoal = xController.atSetpoint() && 
                        yController.atSetpoint() && 
                        rotController.atSetpoint();
        
        boolean timedOut = elapsed > TIMEOUT_SECONDS;
        
        if (atGoal) System.out.println("✅ Reached goal!");
        if (timedOut) System.out.println("⏱️ Timeout!");
        
        return atGoal || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
    }
}