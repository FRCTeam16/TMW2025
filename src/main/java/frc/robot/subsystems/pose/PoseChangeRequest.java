package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems;

public class PoseChangeRequest {
    private final Pose2d targetPose;
    private final boolean usePoseRotation;

    public PoseChangeRequest(Pose2d targetPose, boolean usePoseRotation) {
        this.targetPose = targetPose;
        this.usePoseRotation = usePoseRotation;
    }

    public PoseChangeRequest(Pose2d targetPose) {
        this(targetPose, false);
    }

    public Pose2d getPose() {
        return new Pose2d(targetPose.getTranslation(),
                usePoseRotation ? targetPose.getRotation() :
                        Subsystems.swerveSubsystem.getState().Pose.getRotation());
    }

    public boolean isUsePoseRotation() { return usePoseRotation; }
}
