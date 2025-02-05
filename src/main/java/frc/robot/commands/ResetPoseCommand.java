package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class ResetPoseCommand extends Command {
    private final Pose2d pose;


    public ResetPoseCommand(Pose2d pose) {
        this.pose = pose;
    }

    public ResetPoseCommand(Translation2d translation) {
        this.pose = new Pose2d(translation, Subsystems.swerveSubsystem.getPigeon2().getRotation2d());
    }

    public ResetPoseCommand() {
        this(new Pose2d(0, 0, Subsystems.swerveSubsystem.getPigeon2().getRotation2d()));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
         Subsystems.swerveSubsystem.resetPose(pose);
    }
}
