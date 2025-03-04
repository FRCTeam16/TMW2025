package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;

public class PoseChangeRequest extends AbstractPoseChangeRequest {
    private final Pose2d targetPose;
    private final boolean usePoseRotation;

    /**
     * Change the robot's pose to the target pose
     * @param targetPose
     * @param usePoseRotation
     */
    public PoseChangeRequest(Pose2d targetPose, boolean usePoseRotation) {
        this.targetPose = targetPose;
        this.usePoseRotation = usePoseRotation;
    }

    /**
     * Change the robot's pose to the target pose but ignoring the passed in rotation
     * @param targetPose
     */
    public PoseChangeRequest(Pose2d targetPose) {
        this(targetPose, false);
    }

    @Override
    void execute() {
        if (usePoseRotation) {
            BSLogger.log("PoseChangeRequest", "resetting to: " + targetPose);
            Subsystems.swerveSubsystem.resetPose(targetPose);
        } else {
            BSLogger.log("PoseChangeRequest", "setting translation to: " + targetPose.getTranslation());
            Subsystems.swerveSubsystem.resetTranslation(targetPose.getTranslation());
        }
    }
}