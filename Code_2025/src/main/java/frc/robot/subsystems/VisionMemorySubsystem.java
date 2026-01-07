package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

/**
 * VisionMemorySubsystem - Stores last known robot position
 * 
 * Purpose: Keep track of robot position when Limelight temporarily loses target
 * or on startup before first MegaTag2 reading.
 * 
 * Keeps memory for 3 seconds, then marks as stale.
 */
public class VisionMemorySubsystem extends SubsystemBase {
    
    private final LimelightSubsystem limelight;
    
    private Pose2d lastValidPose = null;
    private long lastUpdateTime = 0;
    private int lastSeenTagID = -1;
    
    // How long to keep memory valid (milliseconds)
    private static final long MEMORY_VALIDITY_DURATION = 3000;
    
    // How often to update memory (milliseconds)
    private static final long UPDATE_INTERVAL = 100;
    private long lastMemoryUpdate = 0;
    
    public VisionMemorySubsystem(LimelightSubsystem limelight) {
        this.limelight = limelight;
    }
    
    @Override
    public void periodic() {
        long currentTime = System.currentTimeMillis();
        
        // Update memory if enough time has passed
        if (currentTime - lastMemoryUpdate > UPDATE_INTERVAL) {
            updateMemory();
            lastMemoryUpdate = currentTime;
        }
        
        publishToSmartDashboard();
    }
    
    /**
     * Update memory with current Limelight data
     * Only stores data if fresh from MegaTag2
     */
    private void updateMemory() {
        if (limelight.isMegaTagDataFresh()) {
            lastValidPose = limelight.getRobotPose();
            lastUpdateTime = System.currentTimeMillis();
            lastSeenTagID = limelight.getLastTagID();
        }
    }
    
    /**
     * Get stored robot position
     * Returns current if fresh, memory if stale but recent, empty if too old
     */
    public Optional<Pose2d> getStoredRobotPose() {
        if (lastValidPose == null) {
            return Optional.empty();
        }
        
        long timeSinceMemory = System.currentTimeMillis() - lastUpdateTime;
        
        if (timeSinceMemory < MEMORY_VALIDITY_DURATION) {
            return Optional.of(lastValidPose);
        }
        
        // Memory expired
        return Optional.empty();
    }
    
    /**
     * Check if stored position is still valid
     */
    public boolean hasValidMemory() {
        return getStoredRobotPose().isPresent();
    }
    
    /**
     * Get time since memory was last updated (milliseconds)
     */
    public long getMemoryAge() {
        if (lastValidPose == null) {
            return Long.MAX_VALUE;
        }
        return System.currentTimeMillis() - lastUpdateTime;
    }
    
    /**
     * Get last seen April tag ID
     */
    public int getLastSeenTagID() {
        return lastSeenTagID;
    }
    
    /**
     * Manually set position (for testing or emergency initialization)
     */
    public void setManualPosition(Pose2d pose) {
        lastValidPose = pose;
        lastUpdateTime = System.currentTimeMillis();
    }
    
    /**
     * Publish telemetry to SmartDashboard
     */
    private void publishToSmartDashboard() {
        SmartDashboard.putBoolean("Has Valid Memory", hasValidMemory());
        SmartDashboard.putNumber("Memory Age (ms)", getMemoryAge());
        
        if (lastValidPose != null) {
            SmartDashboard.putNumber("Memory X", lastValidPose.getX());
            SmartDashboard.putNumber("Memory Y", lastValidPose.getY());
            SmartDashboard.putNumber("Memory Heading", lastValidPose.getRotation().getDegrees());
        }
        
        SmartDashboard.putNumber("Last Seen Tag", lastSeenTagID);
    }
}
