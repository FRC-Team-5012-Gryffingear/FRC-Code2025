package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * LimelightSubsystem - MegaTag2 Vision Input
 * 
 * Uses LimelightHelpers to get MegaTag2 pose
 * Provides vision measurements to pose estimator
 */
public class LimelightSubsystem extends SubsystemBase {
    
    private static final String LIMELIGHT_NAME = "limelight-senior";
    
    private Pose2d lastValidPose = new Pose2d();
    private int lastValidTagCount = 0;
    private double lastMeasurementTimestamp = 0;
    
    public LimelightSubsystem() {
        // Configure Limelight
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, 0);  // AprilTag pipeline
    }
    
    /**
     * Update robot orientation for MegaTag2
     * MUST be called every frame with current robot yaw
     */
    public void updateRobotOrientation(double robotYawDegrees) {
        // Tell Limelight the robot's current orientation
        // This is CRITICAL for MegaTag2 accuracy
        LimelightHelpers.SetRobotOrientation(
            LIMELIGHT_NAME,
            robotYawDegrees,  // Robot yaw in degrees
            0, 0, 0, 0, 0     // Other angles (not needed for MT2)
        );
    }
    
    /**
     * Get MegaTag2 pose estimate
     * Returns null if no valid target
     */
    public LimelightHelpers.PoseEstimate getMegaTag2Pose() {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
            LIMELIGHT_NAME
        );
        
        // Validity checks
        if (mt2 == null || mt2.tagCount == 0) {
            return null;
        }
        
        // Check if the measurement is recent and valid
        lastValidPose = mt2.pose;
        lastValidTagCount = mt2.tagCount;
        lastMeasurementTimestamp = mt2.timestampSeconds;
        
        return mt2;
    }
    
    public Pose2d getLastValidPose() {
        return lastValidPose;
    }
    
    public int getLastTagCount() {
        return lastValidTagCount;
    }
    
    public double getLastMeasurementTimestamp() {
        return lastMeasurementTimestamp;
    }
    
    public boolean hasValidTarget() {
        return lastValidTagCount > 0;
    }

    public int getTagID(){
        return (int) LimelightHelpers.getFiducialID(LIMELIGHT_NAME);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Limelight Tag Count", lastValidTagCount);
        SmartDashboard.putBoolean("Limelight Has Target", hasValidTarget());
    }
}
