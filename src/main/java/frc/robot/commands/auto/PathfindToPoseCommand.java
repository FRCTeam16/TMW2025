package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class PathfindToPoseCommand extends Command {
    Command pathCommand = null;
    public PathfindToPoseCommand() {
        addRequirements(Subsystems.swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d targetPose = new Pose2d(3.0, 2.25, Rotation2d.fromDegrees(45));
        PathConstraints pathConstraints = new PathConstraints(1.0, 1.0, 1.0, 1.0);
        pathCommand = AutoBuilder.pathfindToPose(targetPose, pathConstraints);
        pathCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }
}
