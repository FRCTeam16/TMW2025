package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;

public class ResetPoseCommand extends Command {
    private final Pose2d pose;

    public ResetPoseCommand() {
        this.pose = null;
    }

    public ResetPoseCommand(Translation2d translation) {
        this.pose = new Pose2d(translation, Subsystems.swerveSubsystem.getPigeon2().getRotation2d());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        Robot.poseUpdates.add(pose);    // FIXME: null check
        if (pose != null) {
            BSLogger.log("ResetPoseCommand", "Resetting pose to: " + pose);
            Subsystems.swerveSubsystem.resetPose(pose);
        } else {
            BSLogger.log("RestPoseCommand", "Zeroing Pose");
            Subsystems.swerveSubsystem.tareEverything();
        }
    }
}
