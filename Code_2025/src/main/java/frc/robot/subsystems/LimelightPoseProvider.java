package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;
import frc.robot.LimelightHelpers;

/**
 * LimelightPoseProvider
 * 
 * Implements the 7028 video pattern:
 * - Provides raw vision measurements from Limelight MegaTag2
 * - Manages SwerveDrivePoseEstimator to fuse odometry + vision
 * - Updates Limelight orientation (REQUIRED every frame!)
 */
public class LimelightPoseProvider extends SubsystemBase {
    
    private static final String LL_NAME = "limelight-daniel";
    
    // Camera mount offsets (CONFIGURE FOR YOUR ROBOT!)
    private static final double CAMERA_FORWARD_OFFSET = 0.0;  // meters
    private static final double CAMERA_RIGHT_OFFSET = 0.0;    // meters  
    private static final double CAMERA_HEIGHT = 0.1905;          // meters
    
    private final SwerveSubsystem swerveSubsystem;
    private final SwerveDrivePoseEstimator poseEstimator;
    private double lastVisionTimestamp = 0;
    
    public LimelightPoseProvider(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        
        // Initialize pose estimator (follows 7028 video exactly)
        this.poseEstimator = new SwerveDrivePoseEstimator(
            swerveSubsystem.getSwerveDrive().kinematics,
            swerveSubsystem.getSwerveDrive().getOdometryHeading(),
            swerveSubsystem.getSwerveDrive().getModulePositions(),
            new Pose2d(0, 0, new Rotation2d()),
            // Odometry standard deviations
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            // Vision standard deviations (be conservative - trust odometry by default)
            VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(9))
        );
    }
    
    /**
     * CRITICAL: Must be called every frame with current gyro heading
     * This tells Limelight our orientation so it can calculate pose correctly
     * 
     * Call from Robot.robotPeriodic() or command periodic
     */
    public void updateRobotOrientation(double gyroHeadingDegrees) {
        LimelightHelpers.SetRobotOrientation(
            LL_NAME,
            gyroHeadingDegrees,
            0, 0,              // Roll/Pitch rates (ignored for ground robots)
            0, 0, 0            // Angular velocities
        );
    }
    
    /**
     * Get the filtered pose estimate (use this for commands)
     * This is what the 7028 videos use - the PoseEstimator output
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public double getTopDownX(){
        return LimelightHelpers.getCameraPose3d_TargetSpace(LL_NAME).getX();
    }
    

    public double getTopDownZ(){
        return LimelightHelpers.getCameraPose3d_TargetSpace(LL_NAME).getZ();
    }

    public double getTopDownYaw(){
        return LimelightHelpers.getCameraPose3d_TargetSpace(LL_NAME).getRotation().getY();
    }

    /**
     * Get raw MegaTag2 measurement if needed
     * Includes tag count and latency for debugging
     */
    public LimelightHelpers.PoseEstimate getRawVisionMeasurement() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_NAME);
    }
    
    @Override
    public void periodic() {
        // Step 1: Update odometry (encoder + gyro data)
        poseEstimator.update(
            swerveSubsystem.getSwerveDrive().getOdometryHeading(),
            swerveSubsystem.getSwerveDrive().getModulePositions()
        );
        
        // Step 2: Get vision measurement from Limelight MegaTag2
        LimelightHelpers.PoseEstimate visionMeasurement = getRawVisionMeasurement();
        
        // Step 3: Feed vision to estimator (only if valid and new)
        if (visionMeasurement != null && visionMeasurement.tagCount >= 1) {
            double timestamp = visionMeasurement.timestampSeconds;
    
            // Only add if this is new data (different timestamp)
            if (timestamp != lastVisionTimestamp) {
                lastVisionTimestamp = timestamp;

                
                // Add to pose estimator (follows 7028 video pattern)
                poseEstimator.addVisionMeasurement(
                    visionMeasurement.pose,
                    timestamp
                );
                
                SmartDashboard.putBoolean("LL/Vision Valid", true);
                SmartDashboard.putNumber("LL/Tag Count", visionMeasurement.tagCount);
                SmartDashboard.putNumber("LL/Latency", visionMeasurement.latency);
            }
        } else {
            SmartDashboard.putBoolean("LL/Vision Valid", false);
        }
        
        // Dashboard telemetry
        Pose2d estimatedPose = getEstimatedPose();
        SmartDashboard.putNumber("LL/Est X", estimatedPose.getX());
        SmartDashboard.putNumber("LL/Est Y", estimatedPose.getY());
        SmartDashboard.putNumber("LL/Est Heading", estimatedPose.getRotation().getDegrees());
    }
}
