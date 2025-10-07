package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

import java.lang.invoke.LambdaConversionException;
import java.util.function.BooleanSupplier;


/**
 * Vision subsystem that integrates Limelight AprilTag detection with PathPlanner
 * for dynamic robot pose estimation and path adjustment
 */
public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final DriveSubsystem driveSubsystem;
    private final BooleanSupplier autoRunSupplier;
    
    // AprilTag field layout configuration
    private static final double APRILTAG_POSITION_TOLERANCE = 0.1; // meters
    private static final double APRILTAG_CONFIDENCE_THRESHOLD = 0.4;
    
    // Vision update parameters
    private boolean hasValidTarget = false;
    private Pose2d lastValidPose = null;
    private double lastUpdateTime = 0;
    
    public VisionSubsystem(DriveSubsystem driveSubsystem, BooleanSupplier autoRunSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.autoRunSupplier = autoRunSupplier;
        
        // Configure Limelight pipeline for AprilTag detection
        configureLimelight();
    }
    
    private void configureLimelight() {
        // Set pipeline to AprilTag detection (usually pipeline 0)
        limelightTable.getEntry("pipeline").setNumber(0);
        
        // Enable LED control if needed
        limelightTable.getEntry("ledMode").setNumber(0); // 0 = pipeline default
    }
    
    @Override
    public void periodic() {
        // Update pose estimation from AprilTags
        updatePoseFromAprilTags();
        
        // Output telemetry
        publishTelemetry();
    }
    
    /**
     * Gets robot pose from Limelight's MegaTag or AprilTag detection
     */
    private void updatePoseFromAprilTags() {
        // Check if we have a valid target
        double tv = limelightTable.getEntry("tv").getDouble(0);
        hasValidTarget = tv == 1.0;
        
        if (!hasValidTarget) {
            return;
        }
        
        // Get bot pose from Limelight (field-relative coordinates)
        // botpose_wpiblue = [x, y, z, roll, pitch, yaw, latency]
        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        
        if (botpose.length < 6) {
            return;
        }
        
        // Extract pose components
        double x = botpose[0];
        double y = botpose[1];
        double yaw = botpose[5]; // rotation in degrees
        double latency = botpose.length > 6 ? botpose[6] : 0;
        
        // Get tag count and average distance for confidence calculation
        double tagCount = limelightTable.getEntry("ta").getDouble(0); // Area/confidence
        
        // Create Pose2d from Limelight data
        Pose2d visionPose = new Pose2d(
            x,
            y,
            Rotation2d.fromDegrees(yaw)
        );
        
        // Calculate timestamp with latency compensation
        double timestamp = (System.currentTimeMillis() / 1000.0) - (latency / 1000.0);
        
        // Determine confidence based on tag area and other factors
        double confidence = calculateVisionConfidence(tagCount, x, y);
        
        // Update pose estimator if confidence is sufficient
        if (confidence > APRILTAG_CONFIDENCE_THRESHOLD) {
            if (!autoRunSupplier.getAsBoolean()) {
                // Update the drive subsystem's pose estimator
                driveSubsystem.addVisionMeasurement(visionPose, timestamp, confidence);
            
                lastValidPose = visionPose;
                lastUpdateTime = timestamp;
            }
        }
    }
    
    /**
     * Calculates vision measurement confidence based on various factors
     */
    private double calculateVisionConfidence(double tagArea, double x, double y) {
        // Base confidence on tag area (larger = closer = more confident)
        double areaConfidence = Math.min(tagArea / 5.0, 1.0);
        
        // Reduce confidence for poses near field edges (potential reflections)
        double edgeDistance = Math.min(
            Math.min(x, 16.54 - x), // Field width
            Math.min(y, 8.02 - y)   // Field height
        );
        double edgeConfidence = Math.min(edgeDistance / 1.0, 1.0);
        
        return areaConfidence * edgeConfidence;
    }
    
    /**
     * Creates a PathPlanner path to a target pose with AprilTag correction
     * @param targetPose Final desired pose
     * @param constraints Path constraints (max velocity, acceleration)
     * @return Command to execute the path
     */
    public edu.wpi.first.wpilibj2.command.Command createCorrectedPath(
            Pose2d targetPose, 
            PathConstraints constraints) {
        
        // Get current pose (corrected by vision)
        Pose2d currentPose = driveSubsystem.getPose();
        
        // Create dynamic path from current pose to target
        return AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Goal end velocity
        );
    }
    
    /**
     * Adjusts a target pose based on detected AprilTag offsets
     * Useful for game piece pickup or scoring adjustments
     */
    public Pose2d adjustTargetPoseFromVision(Pose2d nominalTarget, int targetAprilTagId) {
        if (!hasValidTarget) {
            return nominalTarget;
        }
        
        // Get the detected AprilTag ID
        double detectedId = limelightTable.getEntry("tid").getDouble(-1);
        
        if (detectedId != targetAprilTagId) {
            return nominalTarget;
        }
        
        // Get the offset from the expected tag position
        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[7]);
        
        if (botpose.length < 6) {
            return nominalTarget;
        }
        
        // Calculate the correction based on vision
        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
        Pose2d currentPose = driveSubsystem.getPose();
        
        // Apply the delta to the target
        Transform2d correction = new Transform2d(
            visionPose.getTranslation().minus(currentPose.getTranslation()),
            visionPose.getRotation().minus(currentPose.getRotation())
        );
        
        return nominalTarget.plus(correction);
    }
    
    /**
     * Creates a path that dynamically updates based on AprilTag detection
     * Useful for precise alignment to scoring positions
     */
    public edu.wpi.first.wpilibj2.command.Command createAdaptivePath(
            Pose2d initialTarget,
            int trackingAprilTagId,
            PathConstraints constraints) {
        
        return edu.wpi.first.wpilibj2.command.Commands.run(() -> {
            // Continuously adjust target based on AprilTag
            Pose2d adjustedTarget = adjustTargetPoseFromVision(initialTarget, trackingAprilTagId);
            
            // Update path in real-time
            // Note: This creates a new path each cycle - in practice, you might
            // want to throttle this or use a different update strategy
            if (hasValidTarget) {
                driveSubsystem.followPath(adjustedTarget, constraints);
            }
        }, driveSubsystem);
    }
    
    /**
     * Gets distance to detected AprilTag
     */
    public double getDistanceToTag() {
        if (!hasValidTarget) {
            return -1.0;
        }
        
        // Calculate distance using camera angles
        double ty = limelightTable.getEntry("ty").getDouble(0);
        double mountAngle = Units.degreesToRadians(20.0); // Adjust for your mount
        double mountHeight = 0.5; // meters - adjust for your robot
        double tagHeight = 1.35; // meters - typical AprilTag height
        
        return (tagHeight - mountHeight) / Math.tan(mountAngle + Units.degreesToRadians(ty));
    }
    
    /**
     * Checks if robot pose has diverged significantly from vision
     * Can trigger automatic correction routines
     */
    public boolean needsPoseCorrection() {
        if (!hasValidTarget || lastValidPose == null) {
            return false;
        }
        
        Pose2d currentPose = driveSubsystem.getPose();
        double poseDelta = currentPose.getTranslation()
            .getDistance(lastValidPose.getTranslation());
        
        return poseDelta > APRILTAG_POSITION_TOLERANCE;
    }
    
    public boolean hasTarget() {
        return hasValidTarget;
    }
    
    public Pose2d getLastValidVisionPose() {
        return lastValidPose;
    }
    
    private void publishTelemetry() {
        SmartDashboard.putBoolean("Vision/HasTarget", hasValidTarget);
        SmartDashboard.putNumber("Vision/LastUpdate", lastUpdateTime);
        
        if (lastValidPose != null) {
            SmartDashboard.putNumber("Vision/X", lastValidPose.getX());
            SmartDashboard.putNumber("Vision/Y", lastValidPose.getY());
            SmartDashboard.putNumber("Vision/Rotation", lastValidPose.getRotation().getDegrees());
        }
        
        if (hasValidTarget) {
            SmartDashboard.putNumber("Vision/Distance", getDistanceToTag());
            SmartDashboard.putNumber("Vision/TagID", 
                limelightTable.getEntry("tid").getDouble(-1));
        }
    }
}
