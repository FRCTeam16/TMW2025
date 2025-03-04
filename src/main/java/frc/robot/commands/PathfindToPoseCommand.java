package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;

/**
 * Command to pathfind to a specified pose
 */
public class PathfindToPoseCommand extends Command {
    StructPublisher<Pose2d> targetPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("PathfindToPose/targetPose", Pose2d.struct).publish();

    Command pathCommand = null;
    private Pose2d targetPose;
    private PathConstraints pathConstraints = new PathConstraints(0.5, 1.0, 1.0, 1.0);

    protected PathfindToPoseCommand() {
        addRequirements(Subsystems.swerveSubsystem);
    }

    public PathfindToPoseCommand(Pose2d targetPose) {
        this.targetPose = targetPose;
        addRequirements(Subsystems.swerveSubsystem);
    }

    public void setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    public PathfindToPoseCommand withConstraints(PathConstraints constraints) {
        this.pathConstraints = constraints;
        return this;
    }

    @Override
    public void initialize() {
        targetPublisher.set(targetPose);
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
