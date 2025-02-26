package frc.robot.hci.control;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Drives to a pose with a PPHolonomicController
 */
public class DriveToPoseWithPPHC extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;
    private final PPHolonomicDriveController controller;
    private final double maxVelocity = 4.0; // m/s
    private final SwerveRequest.ApplyRobotSpeeds speeds = new SwerveRequest.ApplyRobotSpeeds();

    public DriveToPoseWithPPHC(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.controller = new PPHolonomicDriveController(
                new PIDConstants(12, 0, 0),  // Translation PID
                new PIDConstants(7, 0, 0),   // Rotation PID
                maxVelocity                  // Max velocity
        );
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;


        // Create a trajectory state for the target pose
        PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
        targetState.pose = targetPose;
        targetState.linearVelocity = 0.0;
        targetState.heading = targetPose.getRotation();
        targetState.fieldSpeeds = new ChassisSpeeds();

        // Calculate robot-relative speeds
        ChassisSpeeds robotSpeeds = controller.calculateRobotRelativeSpeeds(
                currentPose,
                targetState
        );

        // Apply speeds using the same request type as your PathPlanner config
        drivetrain.setControl(speeds.withSpeeds(robotSpeeds));
    }

    @Override
    public boolean isFinished() {
        double poseTolerance = 0.1; // meters
        double rotTolerance = 5.0; // degrees

        Pose2d currentPose = drivetrain.getState().Pose;
        double poseError = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double rotError = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees());

        return poseError < poseTolerance && rotError < rotTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(speeds.withSpeeds(new ChassisSpeeds()));
    }
}