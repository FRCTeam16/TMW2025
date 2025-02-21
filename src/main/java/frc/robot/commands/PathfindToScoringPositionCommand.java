package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.commands.auto.PathfindToPoseCommand;
import frc.robot.subsystems.scoring.TargetPose;
import frc.robot.subsystems.vision.VisionTypes;

import java.util.Optional;

/**
 * Debug/Testing command to pathfind to a scoring position
 */
public class PathfindToScoringPositionCommand extends Command {

    private final boolean isLeft;
    Alert invalidStateAlert = new Alert("Invalid PathfindToScoringPositionCommand Scoring Request", Alert.AlertType.kWarning);
    private PathfindToPoseCommand pathfindToPoseCommand;

    public PathfindToScoringPositionCommand(boolean isLeft) {
         addRequirements(Subsystems.swerveSubsystem);
         this.isLeft = isLeft;
    }

    @Override
    public void initialize() {
        Optional<VisionTypes.TargetInfo> targetInfo = Subsystems.visionSubsystem.getTargetInfo();
        if (targetInfo.isEmpty()) {
            invalidStateAlert.set(true);
            return;
        }
        Optional<Pose2d> optionalTargetPose = TargetPose.getScoringPoseForTag(targetInfo.get().aprilTagID(), isLeft);

        if (optionalTargetPose.isEmpty()) {
            invalidStateAlert.set(true);
            return;
        }
        pathfindToPoseCommand = new PathfindToPoseCommand(optionalTargetPose.get());
        pathfindToPoseCommand.initialize();
    }

    @Override
    public void execute() {
        pathfindToPoseCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        pathfindToPoseCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return pathfindToPoseCommand.isFinished();
    }
}
