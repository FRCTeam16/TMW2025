package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.VisionTypes;

import java.util.Optional;

/**
 * Debug/Testing command to pathfind to a scoring position
 */
@Deprecated
public class PathfindToScoringPositionCommand extends Command {

    private final boolean isLeft;
    private final StructPublisher<Pose2d> posePublisher;
    Alert invalidStateAlert = new Alert("Invalid PathfindToScoringPositionCommand Scoring Request", Alert.AlertType.kWarning);
    private PathfindToPoseCommand pathfindToPoseCommand;


    public PathfindToScoringPositionCommand(boolean isLeft) {
        addRequirements(Subsystems.swerveSubsystem);
        this.isLeft = isLeft;
        posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("PathfindToScorePose", Pose2d.struct).publish();
    }

    @Override
    public void initialize() {
        Optional<VisionTypes.TargetInfo> targetInfo = Subsystems.visionSubsystem.getTargetInfo();
        if (targetInfo.isEmpty()) {
            invalidStateAlert.set(true);
            return;
        }
        Optional<Pose2d> optionalTargetPose = Subsystems.aprilTagUtil.getScoringPoseForTag(targetInfo.get().aprilTagID(), isLeft);

        if (optionalTargetPose.isEmpty()) {
            invalidStateAlert.set(true);
            return;
        }

        posePublisher.set(optionalTargetPose.get());

        pathfindToPoseCommand = new PathfindToPoseCommand(optionalTargetPose.get());
        pathfindToPoseCommand.initialize();
    }

    @Override
    public void execute() {
        if (pathfindToPoseCommand != null) {
            pathfindToPoseCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (pathfindToPoseCommand != null) {
            pathfindToPoseCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return pathfindToPoseCommand == null || pathfindToPoseCommand.isFinished();
    }
}
