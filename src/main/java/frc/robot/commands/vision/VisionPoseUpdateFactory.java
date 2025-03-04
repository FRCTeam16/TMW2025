package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.subsystems.pose.PoseChangeRequest;

public class VisionPoseUpdateFactory {

    public static Command resetFromMainPoseEstimator() {
        return Commands.runOnce(() -> {
            Pose2d pose = Subsystems.visionOdometryUpdater.getEstimatedPose();
            Subsystems.poseManager.pushRequest(new PoseChangeRequest(pose));
        });
    }

    public static Command resetTranslationFromMainPoseEstimator() {
        return Commands.runOnce(() -> {
            Pose2d pose = Subsystems.visionOdometryUpdater.getEstimatedPose();
            Subsystems.poseManager.pushRequest(
                    new PoseChangeRequest(pose));
        });
    }

}
