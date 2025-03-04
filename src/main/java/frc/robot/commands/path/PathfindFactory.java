package frc.robot.commands.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.commands.vision.LimelightBasedAlignmentCommand;
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
            noPoseAlert.set(false);
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
            targetPublisher.set(targetPose);
            return new PathfindToPoseCommand(targetPose);
        }, Set.of(Subsystems.swerveSubsystem));
    }

    public static Command pidDriveToAprilTag(int aprilTagId, boolean isLeft) {
        return Commands.defer(() -> {
            noPoseAlert.set(false);
            Pose2d tagPose = Subsystems.aprilTagUtil.getTagPose2d(aprilTagId).orElse(null);
            Pose2d targetPose = Subsystems.aprilTagUtil.getScoringPoseForTag(aprilTagId, isLeft).orElse(null);
            if (targetPose == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            tagPublisher.set(tagPose);
            targetPublisher.set(targetPose);
            return new DriveToPoseCommand(targetPose);
        }, Set.of(Subsystems.swerveSubsystem));
    }


    public static Command pidDriveToVisibleAprilTag(boolean isLeft) {
        return Commands.defer(() -> {
            noPoseAlert.set(false);
            VisionTypes.TargetInfo targetInfo = Subsystems.visionSubsystem.getTargetInfo().orElse(null);
            if (targetInfo == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            Pose2d tagPose = Subsystems.aprilTagUtil.getTagPose2d(targetInfo.aprilTagID()).orElse(null);
            Pose2d targetPose = Subsystems.aprilTagUtil.getScoringPoseForTag(targetInfo.aprilTagID(), isLeft).orElse(null);
            if (targetPose == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            tagPublisher.set(tagPose);
            targetPublisher.set(targetPose);
            return new DriveToPoseCommand(targetPose);
        }, Set.of(Subsystems.swerveSubsystem));
    }

    public static Command holonomicDriveToVisibleAprilTag(boolean isLeft) {
        return Commands.defer(() -> {
            noPoseAlert.set(false);
            VisionTypes.TargetInfo targetInfo = Subsystems.visionSubsystem.getTargetInfo().orElse(null);
            if (targetInfo == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            Pose2d tagPose = Subsystems.aprilTagUtil.getTagPose2d(targetInfo.aprilTagID()).orElse(null);
            Pose2d targetPose = Subsystems.aprilTagUtil.getScoringPoseForTag(targetInfo.aprilTagID(), isLeft).orElse(null);
            if (targetPose == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            tagPublisher.set(tagPose);
            targetPublisher.set(targetPose);
            return new HolonomicDriveToPoseCommand(targetPose);
        }, Set.of(Subsystems.swerveSubsystem));
    }

    public static Command pidDriveProfiledToAprilTag(boolean isLeft) {
        return Commands.defer(() -> {
            VisionTypes.TargetInfo targetInfo = Subsystems.visionSubsystem.getTargetInfo().orElse(null);
            if (targetInfo == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            Pose2d tagPose = Subsystems.aprilTagUtil.getTagPose2d(targetInfo.aprilTagID()).orElse(null);
            Pose2d targetPose = Subsystems.aprilTagUtil.getScoringPoseForTag(targetInfo.aprilTagID(), isLeft).orElse(null);
            if (targetPose == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            tagPublisher.set(tagPose);
            targetPublisher.set(targetPose);
            return new DriveToPoseProfiledCommand(targetPose);
        }, Set.of(Subsystems.swerveSubsystem));
    }

    public static Command limelightAlignToVisibleAprilTag(boolean isLeft) {
        return Commands.defer(() -> {
            VisionTypes.TargetInfo targetInfo = Subsystems.visionSubsystem.getTargetInfo().orElse(null);
            if (targetInfo == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            Pose2d tagPose = Subsystems.aprilTagUtil.getTagPose2d(targetInfo.aprilTagID()).orElse(null);
            Pose2d targetPose = Subsystems.aprilTagUtil.getScoringPoseForTag(targetInfo.aprilTagID(), isLeft).orElse(null);
            if (targetPose == null) {
                noPoseAlert.set(true);
                return Commands.none();
            }
            tagPublisher.set(tagPose);
            targetPublisher.set(targetPose);
            return new LimelightBasedAlignmentCommand(isLeft);
        }, Set.of(Subsystems.swerveSubsystem));
    }
}