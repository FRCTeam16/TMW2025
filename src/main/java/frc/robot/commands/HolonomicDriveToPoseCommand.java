package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ProfiledRotationController;
import frc.robot.subsystems.TranslationController;
import frc.robot.subsystems.pose.PoseChangeRequest;
import frc.robot.util.BSLogger;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class HolonomicDriveToPoseCommand extends Command {
    // Tolerances
    private static final double DISTANCE_TOLERANCE = 0.02; // meters
    private static final double ANGLE_TOLERANCE = 2;
    private final CommandSwerveDrivetrain drivetrain = Subsystems.swerveSubsystem;
    private final Pose2d targetPose;

    // PID controllers for translation and rotation
    private final TranslationController distancePID = Subsystems.translationController;
    private final ProfiledRotationController anglePID = Subsystems.profiledRotationController;
    private final HolonomicDriveController driveController;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public HolonomicDriveToPoseCommand(Pose2d targetPose) {
        this.targetPose = targetPose;
        distancePID.setTolerance(DISTANCE_TOLERANCE);
        anglePID.setTolerance(ANGLE_TOLERANCE);

        driveController = new HolonomicDriveController(
                distancePID,
                distancePID,
                anglePID
        );
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset PID controllers at the start
        anglePID.reset(Subsystems.swerveSubsystem.getState().Pose.getRotation().getDegrees());
    }

    @Override
    public void execute() {
        System.out.println("TEST TEST TEST");
        // When we are close to the target we want to constantly update
        // our pose based on the camera
        Optional<Double> averageDistance = Subsystems.visionOdometryUpdater.getTargetDistance();
        if (averageDistance.isPresent() && averageDistance.get() < 2.0) {
            Subsystems.poseManager.pushRequest(new PoseChangeRequest(Subsystems.visionOdometryUpdater.getEstimatedPose()));
        }

        // Obtain the current robot pose from the drivetrain
        Pose2d currentPose = drivetrain.getState().Pose;
        Rotation2d targetAngle = targetPose.getRotation();

        // Calculate the chassis speeds using the HolonomicDriveController
        ChassisSpeeds chassisSpeeds = driveController.calculate(
                currentPose,
                targetPose,
                0.0, // Desired linear velocity (can be set to a specific value if needed)
                targetAngle
        );

        // Log the chassis speeds
        BSLogger.log("DriveToPoseCommand", "Chassis Speeds: " + chassisSpeeds);

        // Apply the calculated speeds to the drivetrain
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(chassisSpeeds));
    }

    @Override
    public boolean isFinished() {
        // The command ends when both translation and rotation are within tolerance.
//        driveController.atReference()
        return distancePID.atSetpoint() && anglePID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        BSLogger.log("DriveToPoseCommand", "Ending - interrupted: " + interrupted);
        // Stop the robot when the command ends or is interrupted.
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(
                new ChassisSpeeds(0.0, 0.0, 0.0)));
    }
}
