package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.LimelightPoseEstimator;

public class UpdateRobotPoseFromVision {
    public static Command resetFromMainPoseEstimator() {
        return Commands.runOnce(() -> {
            Pose2d pose = Subsystems.visionOdometryUpdater.getEstimatedPose();
            Robot.poseUpdates.add(pose);
                    // LOCKS
//            Subsystems.swerveSubsystem.resetPose(Subsystems.visionOdometryUpdater.getEstimatedPose())
        });
    }

    @Deprecated
    public static Command resetFromLimelightPoseEstimator(String limelightName) {
        return Commands.none();
//        return Commands.runOnce(() -> {
//            Subsystems.visionOdometryUpdater.getVisionPoseEstimators().stream()
//                    .filter(visionPoseEstimator -> visionPoseEstimator.getLimelightName().equals(limelightName))
//                    .map(LimelightPoseEstimator::getLastPoseEstimate)
//                    .map(poseEstimate -> poseEstimate.pose)
//                    .findFirst()
//                    .ifPresent(pose2d -> Subsystems.swerveSubsystem.resetPose(pose2d));
//        }, Subsystems.swerveSubsystem);
    }

}
