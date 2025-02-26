package frc.robot.subsystems.vision;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Subsystems;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

/**
 * Class that returns the Limelight estimated pose measurements from AprilTags.
 */
public class LimelightPoseEstimator implements Sendable {
    private final boolean useMegaTag2; // set to false to use MegaTag1
    private final String limelightName;
    private final StructPublisher<Pose2d> publisher;
    private LimelightHelpers.PoseEstimate lastPoseEstimate = null;

    /**
     * Constructs a LimelightPoseEstimator with the specified Limelight name and MegaTag2 usage.
     *
     * @param limelightName The name of the Limelight camera.
     * @param useMegaTag2   Whether to use MegaTag2 for pose estimation.
     */
    public LimelightPoseEstimator(String limelightName, boolean useMegaTag2) {
        this.limelightName = limelightName;
        this.useMegaTag2 = useMegaTag2;
        this.publisher = NetworkTableInstance.getDefault()
                .getStructTopic("LimelightPoseEstimator/" + limelightName + "/pose", Pose2d.struct).publish();
    }

    /**
     * Constructs a LimelightPoseEstimator with the specified Limelight name.
     * Defaults to using MegaTag2 for pose estimation.
     *
     * @param limelightName The name of the Limelight camera.
     */
    public LimelightPoseEstimator(String limelightName) {
        this(limelightName, true);
    }

    public String getLimelightName() {
        return limelightName;
    }

    public LimelightHelpers.PoseEstimate getLastPoseEstimate() {
        return lastPoseEstimate;
    }

    /**
     * Estimates the robot pose using the current robot yaw angle.
     *
     * @param currentRobotYaw The current yaw angle of the robot.
     * @return An Optional containing the pose estimate if available, otherwise an empty Optional.
     */
    public Optional<LimelightHelpers.PoseEstimate> estimatePose(Angle currentRobotYaw) {
        Optional<LimelightHelpers.PoseEstimate> poseEstimate;
        if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation(limelightName, currentRobotYaw.in(Degrees), 0, 0, 0, 0, 0);
            poseEstimate = doMegaTag2Estimate();
        } else {
            poseEstimate = doMegaTag1Estimate();
        }

        // Filters out pose estimates if the tag count is zero
        if (poseEstimate.isPresent() && poseEstimate.get().tagCount == 0) {
            return Optional.empty();
        }
        // Filters out pose estimates if the robot is moving too fast
        StatusSignal<AngularVelocity> angularVelocity = Subsystems.swerveSubsystem.getPigeon2().getAngularVelocityZWorld();
        if (Math.abs(angularVelocity.getValue().in(DegreesPerSecond)) > 100) { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            return Optional.empty();
        }

        if (poseEstimate.isPresent()) {
            LimelightHelpers.PoseEstimate estimate = poseEstimate.get();
            this.lastPoseEstimate = estimate;

            // Publish pose estimate to SmartDashboard
            publisher.set(estimate.pose);
        }

        return poseEstimate;
    }

    /**
     * Performs pose estimation using MegaTag2.
     *
     * @return An Optional containing the pose estimate if available, otherwise an empty Optional.
     */
    private Optional<LimelightHelpers.PoseEstimate> doMegaTag2Estimate() {
        LimelightHelpers.PoseEstimate visionPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName); // TODO Check red switch here?
        return Optional.ofNullable(visionPoseEstimate);
    }

    /**
     * Performs pose estimation using MegaTag1.
     *
     * @return An Optional containing the pose estimate if available, otherwise an empty Optional.
     */
    private Optional<LimelightHelpers.PoseEstimate> doMegaTag1Estimate() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 == null) {
            return Optional.empty();
        }

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7) {
                return Optional.empty();
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                return Optional.empty();
            }
        }
        if (mt1.tagCount == 0) {
            return Optional.empty();
        }
        return Optional.of(mt1);
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.setSmartDashboardType("LimelightPoseEstimator");
        sendableBuilder.addStringProperty("Limelight Name", () -> limelightName, null);
        sendableBuilder.addBooleanProperty("Use MegaTag2", () -> useMegaTag2, null);
        sendableBuilder.addIntegerProperty("TagCount", () -> lastPoseEstimate != null ? lastPoseEstimate.tagCount : 0, null);
        sendableBuilder.addDoubleProperty("Distance", () -> lastPoseEstimate != null ? lastPoseEstimate.avgTagDist : 0.0, null);
        sendableBuilder.addDoubleProperty("TagArea", () -> lastPoseEstimate != null ? lastPoseEstimate.avgTagArea : 0.0, null);
    }

    public String getName() {
        return limelightName;
    }
}