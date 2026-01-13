package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwervePoseEstimatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import java.util.Optional;

/**
 * AlignToReefCommand - PID-based reef alignment
 * 
 * Uses ChassisSpeeds with YAGSL SwerveSubsystem
 */
public class AlignToReefCommand extends Command {
    
    private final SwerveSubsystem swerve;
    private final SwervePoseEstimatorSubsystem poseEstimator;
    private final LimelightSubsystem limelight;
    private final int targetTagID;
    private final double approachDistance;
    
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;
    
    private static final double X_TOLERANCE = 0.05;
    private static final double Y_TOLERANCE = 0.05;
    private static final double ROT_TOLERANCE = 2;
    private static final double TIMEOUT = 6.0;
    
    private double commandStartTime;
    
    public AlignToReefCommand(
        SwerveSubsystem swerve,
        SwervePoseEstimatorSubsystem poseEstimator,
        LimelightSubsystem limelight,
        int tagID,
        double approachDistance
    ) {
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.limelight = limelight;
        this.targetTagID = tagID;
        this.approachDistance = approachDistance;
        
        this.xController = new PIDController(0.25, 0.0, 0.05);
        this.yController = new PIDController(0.25, 0.0, 0.05);
        this.rotController = new PIDController(0.26, 0.0, 0);
        
        xController.setTolerance(X_TOLERANCE);
        yController.setTolerance(Y_TOLERANCE);
        rotController.setTolerance(ROT_TOLERANCE);
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        commandStartTime = System.currentTimeMillis() / 1000.0;
        SmartDashboard.putString("Alignment Status", "Init");
    }
    
    @Override
    public void execute() {
        Pose2d robotPose = poseEstimator.getPose();
        
        Optional<Pose2d> optTagPose = poseEstimator.getFieldLayout()
            .getTagPose(targetTagID)
            .map(p -> p.toPose2d());
        
        if (!optTagPose.isPresent()) {
            SmartDashboard.putString("Alignment Status", "Tag not found");
            swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));  // YOUR ACTUAL METHOD
            return;
        }
        
        Pose2d tagPose = optTagPose.get();
        
        // Target position: approach distance away from tag
        Pose2d targetPose = tagPose.plus(
            new Transform2d(
                new Translation2d(approachDistance, 0),
                new edu.wpi.first.math.geometry.Rotation2d()
            )
        );
        
        double errorX = targetPose.getX() - robotPose.getX();
        double errorY = targetPose.getY() - robotPose.getY();
        double errorRotDegrees = targetPose.getRotation()
            .minus(robotPose.getRotation())
            .getDegrees() + 40;
        
        // Normalize rotation error
        while (errorRotDegrees > 180) errorRotDegrees -= 360;
        while (errorRotDegrees < -180) errorRotDegrees += 360;
        
        double vx = clamp(xController.calculate(0, errorX), 0, 0.1);
        double vy = clamp(yController.calculate(0, errorY), 0, 0.1);
        double omega = rotController.calculate(0, Math.toRadians(errorRotDegrees));
        
        vx = clamp(vx, -2.0, 2.0);
        vy = clamp(vy, -2.0, 2.0);
        omega = clamp(omega, -Math.PI, Math.PI);
        
        // YOUR ACTUAL DRIVE METHOD (robot-relative)
        swerve.getSwerveDrive().drive(new ChassisSpeeds(vx, vy, omega));
        
        SmartDashboard.putNumber("Align Error X", errorX);
        SmartDashboard.putNumber("Align Error Y", errorY);
        SmartDashboard.putNumber("Align Error Rot", errorRotDegrees);
        SmartDashboard.putString("Alignment Status", "DRIVING");
    }
    
    @Override
    public boolean isFinished() {
        double elapsed = System.currentTimeMillis() / 1000.0 - commandStartTime;
        
        boolean aligned = xController.atSetpoint() 
            && yController.atSetpoint() 
            && rotController.atSetpoint();
        
        boolean timedOut = elapsed > TIMEOUT;
        
        if (aligned || timedOut) {
            swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
            SmartDashboard.putString("Alignment Status", "COMPLETE");
        }
        
        return aligned || timedOut;
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
    }
    
    private double clamp(double value, double min, double max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }
}
