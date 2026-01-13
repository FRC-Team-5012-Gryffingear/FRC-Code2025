package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.VecBuilder;

/**
 * SwervePoseEstimatorSubsystem - Core Robot Localization
 * 
 * Fuses:
 * - Swerve odometry (from getModulePositions)
 * - Gyro rotation
 * - MegaTag2 vision measurements
 */
public class SwervePoseEstimatorSubsystem extends SubsystemBase {
    
    private final SwerveSubsystem swerveSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;
    
    // Standard deviations (how much we trust each measurement)
    private static final double ODOMETRY_X_SD = 0.1;
    private static final double ODOMETRY_Y_SD = 0.1;
    private static final double ODOMETRY_ROT_SD = 0.05;
    
    private static final double VISION_X_SD = 0.5;
    private static final double VISION_Y_SD = 0.5;
    private static final double VISION_ROT_SD = 0.1;
    
    public SwervePoseEstimatorSubsystem(
        SwerveSubsystem swerveSubsystem,
        LimelightSubsystem limelightSubsystem
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        
        // Load field layout
        try {
            this.fieldLayout = AprilTagFieldLayout.loadField(
                AprilTagFields.k2025Reefscape
            );
        } catch (Exception e) {
            throw new RuntimeException("Failed to load field layout", e);
        }
        
        // Create pose estimator using your swerve drive's kinematics
        this.poseEstimator = new SwerveDrivePoseEstimator(
            swerveSubsystem.getSwerveDrive().kinematics,  // YOUR swerve kinematics
            swerveSubsystem.getSwerveDrive().getYaw(),                           // Current heading
            swerveSubsystem.getSwerveDrive().getModulePositions(),  // Current positions
            new Pose2d(),  // Start at origin
            VecBuilder.fill(
                ODOMETRY_X_SD, ODOMETRY_Y_SD, ODOMETRY_ROT_SD
            ),
            VecBuilder.fill(
                VISION_X_SD, VISION_Y_SD, VISION_ROT_SD
            )
        );
    }
    
    @Override
    public void periodic() {
        // Update robot orientation for Limelight FIRST
        limelightSubsystem.updateRobotOrientation(
            swerveSubsystem.getSwerveDrive().getYaw().getDegrees()
        );
        
        // Update pose estimator with odometry
        poseEstimator.update(
            swerveSubsystem.getSwerveDrive().getYaw(),
            swerveSubsystem.getSwerveDrive().getModulePositions()
        );
        
        // Update with vision if available
        LimelightHelpers.PoseEstimate mt2 = limelightSubsystem.getMegaTag2Pose();
        if (mt2 != null && mt2.tagCount > 0) {
            // Add vision measurement to pose estimator
            poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds
            );
        }
        
        publishToSmartDashboard();
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
    
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            swerveSubsystem.getSwerveDrive().getYaw(),
            swerveSubsystem.getSwerveDrive().getModulePositions(),
            pose
        );
    }
    
    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }
    
    private void publishToSmartDashboard() {
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Pose X", pose.getX());
        SmartDashboard.putNumber("Pose Y", pose.getY());
        SmartDashboard.putNumber("Pose Heading", pose.getRotation().getDegrees());
    }
}
