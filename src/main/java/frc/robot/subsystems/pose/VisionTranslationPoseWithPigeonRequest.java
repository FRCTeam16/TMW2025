package frc.robot.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;

public class VisionTranslationPoseWithPigeonRequest extends AbstractPoseChangeRequest {

    @Override
    void execute() {
        Pose2d targetPose = Subsystems.visionOdometryUpdater.getEstimatedPose();
        Pose2d requestedPose = new Pose2d(targetPose.getTranslation(),
                new Rotation2d(Subsystems.swerveSubsystem.getPigeonYaw()));
        BSLogger.log("PoseWithPigeonRequest", "resetting to: " + requestedPose);
        Subsystems.swerveSubsystem.resetPose(requestedPose);
    }
}
