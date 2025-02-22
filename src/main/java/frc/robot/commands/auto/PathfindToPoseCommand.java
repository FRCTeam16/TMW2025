package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

public class PathfindToPoseCommand extends Command {
    private final Pose2d targetPose;
    Command pathCommand = null;
    private PathConstraints pathConstraints = new PathConstraints(0.5, 1.0, 1.0, 1.0);

    public PathfindToPoseCommand(Pose2d targetPose) {
        this.targetPose = targetPose;
        addRequirements(Subsystems.swerveSubsystem);
    }

    public PathfindToPoseCommand withConstraints(PathConstraints constraints) {
        this.pathConstraints = constraints;
        return this;
    }

    @Override
    public void initialize() {
        pathCommand = AutoBuilder.pathfindToPose(targetPose, pathConstraints);
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        pathCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        pathCommand.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }
}
