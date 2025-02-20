package frc.robot.subsystems.scoring;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.VisionTypes;
import frc.robot.util.TimeExpiringValue;

import java.util.Optional;

/**
 * Class to calculate the target pose for scoring
 */
public class TargetPose {
    private static final double SCORING_DISTANCE = 0.5; // robot meters from tag
    private static final double OFFSET_DISTANCE = 0.4; // robot horizontal offset from tag
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);   // US regionals use welded

    TimeExpiringValue<Integer> targetTag = new TimeExpiringValue<>(500);

    public void update() {
        Optional<VisionTypes.TargetInfo> targetInfo = Subsystems.visionSubsystem.getTargetInfo();
        targetInfo.ifPresent(info -> targetTag.set(info.aprilTagID()));
    }

    /**
     * Get the desired scoring pose for a given tag pose
     * @param isLeft Whether the scoring position is on the left or right
     * @return The desired scoring pose
     */
    public Optional<Pose2d> getScoringPoseForTag(boolean isLeft) {
        Integer aprilTagID = targetTag.get().orElse(null);
        if (aprilTagID == null) {
            return Optional.empty();
        }
        Pose3d tagPose = fieldLayout.getTagPose(aprilTagID).orElse(null); // Get pose for tag ID 1
        if (tagPose == null) {
            return Optional.empty();
        }
        Pose2d tagPose2d = tagPose.toPose2d();

        // Calculate perpendicular angle for robot alignment
        Rotation2d scoreAngle = tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180));

        // Calculate offset based on left/right scoring position
        double lateralOffset = isLeft ? -OFFSET_DISTANCE : OFFSET_DISTANCE; // meters

        // Create transform from tag to scoring position
        Transform2d scoreTransform = new Transform2d(
                new Translation2d(-SCORING_DISTANCE, lateralOffset),
                scoreAngle
        );

        // Apply transform to get final scoring position
        return Optional.ofNullable(tagPose2d.transformBy(scoreTransform));
    }

    public Optional<Integer> getTargetTagID() {
        return targetTag.get();
    }


    public boolean isValid() {
        return targetTag.isExpired();
    }
}
