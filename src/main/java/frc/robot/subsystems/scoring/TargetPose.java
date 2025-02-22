package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.*;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.AprilTagAngleLookup;
import frc.robot.subsystems.vision.VisionTypes;
import frc.robot.util.TimeExpiringValue;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;

/**
 * Class to calculate the target pose for scoring
 */
public class TargetPose {
    private static final double SCORING_DISTANCE = 0.5; // robot meters from tag
    private static final double OFFSET_DISTANCE = 0.2; // robot horizontal offset from tag

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
        int aprilTagID = targetTag.get().orElse(-1);
        return getScoringPoseForTag(aprilTagID, isLeft);
    }

    public static Optional<Pose2d> getScoringPoseForTag(int aprilTagID, boolean isLeft) {
        if (aprilTagID < 0) {
            return Optional.empty();
        }

        // Gets tag pose in blue alliance coordinates, we use the x/y values
        Pose3d tagPose = Subsystems.visionSubsystem.getTagPose(aprilTagID).orElse(null); // Get pose for tag ID 1
        if (tagPose == null) {
            return Optional.empty();
        }
        Pose2d tagPose2d = tagPose.toPose2d();

        // Calculate perpendicular angle for robot alignment
        Rotation2d scoreAngle = tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180));
//        Rotation2d scoreAngle   = tagPose2d.getRotation();
//         scoreAngle = Rotation2d.fromDegrees(AprilTagAngleLookup.getFacingAngle(aprilTagID).get().in(Degrees));

        // Calculate offset based on left/right scoring position
        double lateralOffset = isLeft ? -OFFSET_DISTANCE : OFFSET_DISTANCE; // meters

        // Create transform from tag to scoring position
        Transform2d scoreTransform = new Transform2d(
                new Translation2d(-SCORING_DISTANCE, lateralOffset),
                Rotation2d.fromDegrees(0)
        );

        // Apply transform to get final scoring position
        Pose2d translatedTarget = tagPose2d.transformBy(scoreTransform);
        Pose2d target = new Pose2d(translatedTarget.getTranslation(), scoreAngle);
        return Optional.of(target);
    }

    public Optional<Integer> getTargetTagID() {
        return targetTag.get();
    }


    public boolean isValid() {
        return targetTag.isExpired();
    }
}
