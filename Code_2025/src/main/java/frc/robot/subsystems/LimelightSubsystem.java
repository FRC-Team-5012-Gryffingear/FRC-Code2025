package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

/**
 * LimelightSubsystem - MegaTag2 Only
 * 
 * Provides robot position in field coordinates using MegaTag2 localization.
 * No ta/ty calculations. Direct position from Limelight.
 * 
 * Primary data source for autonomous navigation.
 */
public class LimelightSubsystem extends SubsystemBase {
    
    private final NetworkTable limelightTable;
    private final AprilTagFieldLayout fieldLayout;
    
    private Pose2d robotPose = new Pose2d();
    private int primaryTagID = -1;
    private boolean hasTarget = false;
    private long lastMegaTagUpdate = 0;
    
    // MegaTag2 validity threshold (milliseconds)
    private static final long MEGATAG_TIMEOUT = 500;
    
    public LimelightSubsystem() {
        this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight-senior");
        
        // Load 2025 Reefscape field layout
        try {
            this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load field layout", e);
        }
        
        // Configure Limelight for MegaTag2
        limelightTable.getEntry("pipeline").setNumber(0);  // April tag pipeline
        limelightTable.getEntry("ledMode").setNumber(3);   // LED on
    }
    
    @Override
    public void periodic() {
        updateFromMegaTag2();
        publishToSmartDashboard();
    }
    
    /**
     * Update robot position from MegaTag2
     * 
     * botpose format: [x, y, z, roll, pitch, yaw]
     * We use: x, y, yaw
     */
    private void updateFromMegaTag2() {
        double[] botpose = limelightTable.getEntry("botpose_orb_wpiblue")
            .getDoubleArray(new double[6]);
        
        // Check if we got valid data
        if (botpose.length >= 6 && hasValidMegaTagData(botpose)) {
            // Update robot position
            robotPose = new Pose2d(
                botpose[0],                    // x
                botpose[1],                    // y
                Rotation2d.fromDegrees(botpose[6])  // yaw (heading)
            );
            
            lastMegaTagUpdate = System.currentTimeMillis();
            hasTarget = true;
            
            // Get detected April tag ID if available
            primaryTagID = (int) limelightTable.getEntry("tid").getDouble(-1);

            SmartDashboard.putNumber("Value 1 (X)", botpose[0]);
            SmartDashboard.putNumber("Value 2 (Y)", botpose[1]);
            SmartDashboard.putNumber("Value 3 (Z)", botpose[2]);
            SmartDashboard.putNumber("Value 4 (RX)", botpose[3]);
            SmartDashboard.putNumber("Value 5 (RY)", botpose[4]);
            SmartDashboard.putNumber("Value 6 (RZ)", botpose[5]);
        } else {
            // No valid MegaTag2 data
            hasTarget = false;
            primaryTagID = -1;
        }
    }
    
    /**
     * Check if MegaTag2 data is valid
     * Must have reasonable field coordinates
     */
    private boolean hasValidMegaTagData(double[] botpose) {
        // 2025 Reefscape field is ~17m x ~8.2m
        // Accept data within reasonable bounds with margin
        double x = botpose[0];
        double y = botpose[1];
        
        // Check if coordinates are within field bounds (with 1m margin)
        boolean inFieldBounds = x >= -1 && x <= 18 && y >= -1 && y <= 9.2;
        
        return inFieldBounds;
    }
    
    /**
     * Get robot's current position in field coordinates
     * Primary localization source
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }
    
    /**
     * Get April tag position from field layout
     */
    public Optional<Pose2d> getAprilTagPose(int tagID) {
        try {
            Optional<Pose3d> pose3d = fieldLayout.getTagPose(tagID);
            if (pose3d.isPresent()) {
                return Optional.of(pose3d.get().toPose2d());
            }
        } catch (Exception e) {
            SmartDashboard.putString("Limelight Error", "Invalid tag ID: " + tagID);
        }
        return Optional.empty();
    }
    
    /**
     * Check if Limelight currently has target
     */
    public boolean hasValidTarget() {
        return hasTarget;
    }
    
    /**
     * Get last detected April tag ID
     */
    public int getLastTagID() {
        return primaryTagID;
    }
    
    /**
     * Check if MegaTag2 data is recent enough
     */
    public boolean isMegaTagDataFresh() {
        long timeSinceUpdate = System.currentTimeMillis() - lastMegaTagUpdate;
        return timeSinceUpdate < MEGATAG_TIMEOUT;
    }
    
    /**
     * Get time since last MegaTag2 update (milliseconds)
     */
    public long getTimeSinceLastUpdate() {
        return System.currentTimeMillis() - lastMegaTagUpdate;
    }
    
    /**
     * Publish telemetry to SmartDashboard
     */
    private void publishToSmartDashboard() {
        SmartDashboard.putNumber("Robot X", robotPose.getX());
        SmartDashboard.putNumber("Robot Y", robotPose.getY());
        SmartDashboard.putNumber("Robot Heading", robotPose.getRotation().getDegrees());
        SmartDashboard.putBoolean("MegaTag Valid", hasTarget);
        SmartDashboard.putNumber("Last Tag ID", primaryTagID);
        SmartDashboard.putNumber("MegaTag Age (ms)", getTimeSinceLastUpdate());
        

    }
}
