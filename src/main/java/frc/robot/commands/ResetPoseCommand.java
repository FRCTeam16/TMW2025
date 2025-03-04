package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

/**
 * Command to reset the robot's pose. Rotation is maintained, but translation is reset to the specified value.
 */
public class ResetPoseCommand extends Command {
    private final Pose2d pose;

    public ResetPoseCommand() {
        this.pose = null;
    }

    public ResetPoseCommand(Translation2d translation) {
        this.pose = new Pose2d(translation, Subsystems.swerveSubsystem.getPigeon2().getRotation2d());
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        Subsystems.poseManager.requestAlliancePoseReset();
    }
}
