package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.VisionTypes;
import frc.robot.util.TimeExpiringValue;

import java.util.Optional;

/**
 * Class to calculate the target pose for scoring
 */
public class TargetPose {

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
        return Subsystems.aprilTagUtil.getScoringPoseForTag(aprilTagID, isLeft);
    }

    public Optional<Integer> getTargetTagID() {
        return targetTag.get();
    }

    public boolean isValid() {
        return !targetTag.isExpired();
    }
}
