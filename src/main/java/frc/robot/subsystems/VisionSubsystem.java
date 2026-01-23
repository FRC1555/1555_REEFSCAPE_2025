package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d cameraToRobot;

    public VisionSubsystem(String cameraName) {
        // Initialize the PhotonCamera with the name of your camera
        camera = new PhotonCamera(cameraName);

        // Load the AprilTagFieldLayout for the 2023 field
        fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

        // Define the transformation from the camera to the robot's center
        cameraToRobot = new Transform3d(
            new Translation3d(0.3556, 0.0762, 0.3556), // x, y, z offsets in meters
            new Rotation3d(0.0, 0.0, 0.0)    // roll, pitch, yaw in radians
        );
    }

    public boolean hasTarget() {
        // Check if the camera sees a target
        PhotonPipelineResult result = camera.getLatestResult();
        return result.hasTargets();
    }

    public double getTargetYaw() {
        // Get the yaw (horizontal angle) of the best target
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getYaw();
        }
        return 0.0; // Default value if no target is found
    }

    public double getTargetDistance(double pitch) {
        // Calculate distance based on the given target pitch (requires calibration)
        // Replace with your own distance calculation formula
        return calculateDistanceFromPitch(pitch);
    }

    private double calculateDistanceFromPitch(double pitch) {
        // Example formula: distance = (targetHeight - cameraHeight) / tan(cameraAngle + pitch)
        double targetHeight = 2.5; // Replace with the height of your target in meters
        double cameraHeight = 1.0; // Replace with the height of your camera in meters
        double cameraAngle = Math.toRadians(30); // Replace with your camera's mounting angle in radians
        return (targetHeight - cameraHeight) / Math.tan(cameraAngle + Math.toRadians(pitch));
    }

    public Optional<Pose2d> getEstimatedPose() {
    PhotonPipelineResult result = camera.getLatestResult();
    if (result.hasTargets()) {
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        double distance = getTargetDistance(bestTarget.getPitch());
        double yaw = bestTarget.getYaw();

        // Calculate the robot's position relative to the field
        Pose2d estimatedPose = new Pose2d(
            /* x */ distance * Math.cos(Math.toRadians(yaw)),
            /* y */ distance * Math.sin(Math.toRadians(yaw)),
            /* rotation */ new Rotation2d(Math.toRadians(yaw))
        );
        return Optional.of(estimatedPose);
    }
    return Optional.empty();
}

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    public Transform3d getCameraToRobot() {
        return cameraToRobot;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        PhotonPipelineResult result = camera.getLatestResult();

        // Publish whether a target is detected
        SmartDashboard.putBoolean("Has Target", result.hasTargets());

        if (result.hasTargets()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();

            // Publish data about the best target
            SmartDashboard.putNumber("Target Yaw", bestTarget.getYaw());
            SmartDashboard.putNumber("Target Pitch", bestTarget.getPitch());
            SmartDashboard.putNumber("Target Area", bestTarget.getArea());
            SmartDashboard.putNumber("Target Skew", bestTarget.getSkew());
            SmartDashboard.putNumber("Target ID", bestTarget.getFiducialId());

            // Publish the number of targets detected
            SmartDashboard.putNumber("Number of Targets", result.getTargets().size());
        } else {
            // Clear the data if no target is detected
            SmartDashboard.putNumber("Target Yaw", 0.0);
            SmartDashboard.putNumber("Target Pitch", 0.0);
            SmartDashboard.putNumber("Target Area", 0.0);
            SmartDashboard.putNumber("Target Skew", 0.0);
            SmartDashboard.putNumber("Target ID", -1);
            SmartDashboard.putNumber("Number of Targets", 0);
        }
    }
}