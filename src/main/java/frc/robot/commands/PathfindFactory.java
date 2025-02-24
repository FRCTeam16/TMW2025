package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.VisionTypes;

import java.util.Set;

public class PathfindFactory {
    static Alert noPoseAlert = new Alert("PathfindFactory: Unable to determine pose", Alert.AlertType.kError);
    static StructPublisher<Pose2d> tagPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Pathing/tagPose", Pose2d.struct).publish();
    static StructPublisher<Pose2d> targetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Pathing/targetPose", Pose2d.struct).publish();

    public static Command pathfindToAprilTag(int aprilTagId, boolean isLeft) {
        return Commands.defer(() -> {
            Pose2d aprilTagPose = Subsystems.aprilTagUtil.getTagPose2d(aprilTagId).orElse(null);
            Pose2d targetPose = Subsystems.aprilTagUtil.getScoringPoseForTag(aprilTagId, isLeft).orElse(null);
            if (targetPose == null) {
                return Commands.none();
            }
            tagPublisher.set(aprilTagPose);
            targetPublisher.set(targetPose);
            return new PathfindToPoseCommand(targetPose);
        }, Set.of(Subsystems.swerveSubsystem));
    }

    public static Command pathfindToVisibleAprilTag(boolean isLeft) {
        return Commands.defer(() -> {
            VisionTypes.TargetInfo targetInfo = Subsystems.visionSubsystem.getTargetInfo().orElse(null);
            if (targetInfo == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            Pose2d targetPose = Subsystems.aprilTagUtil.getScoringPoseForTag(targetInfo.aprilTagID(), isLeft).orElse(null);
            if (targetPose == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            tagPublisher.set(targetPose);
            targetPublisher.set(targetPose);
            return new PathfindToPoseCommand(targetPose);
        }, Set.of(Subsystems.swerveSubsystem));
    }
}