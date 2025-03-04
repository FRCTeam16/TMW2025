package frc.robot.commands.path;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.TranslationController;
import frc.robot.subsystems.pose.PoseChangeRequest;
import frc.robot.util.BSLogger;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class DriveToPoseCommand extends Command {
    // Tolerances
    private static final double DISTANCE_TOLERANCE = 0.02; // meters
    private static final double ANGLE_TOLERANCE = 2;
    private final CommandSwerveDrivetrain drivetrain = Subsystems.swerveSubsystem;
    private final Pose2d targetPose;

    // PID controllers for translation and rotation
    private final TranslationController distancePID = Subsystems.translationController;
    private final RotationController anglePID = Subsystems.rotationController;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public DriveToPoseCommand(Pose2d targetPose) {
        this.targetPose = targetPose;
// 1.8, 0, 0.04)
        // Configure PID tolerances
        distancePID.setTolerance(DISTANCE_TOLERANCE);
        anglePID.setTolerance(ANGLE_TOLERANCE);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset PID controllers at the start
        distancePID.reset();
        anglePID.reset();

        distancePID.setSetpoint(0);

        Rotation2d targetAngle = targetPose.getRotation();
        anglePID.setSetpoint(targetAngle.getDegrees());
    }

    @Override
    public void execute() {

        // When we are close to the target we want to constantly update
        // out pose based on the camera

        Optional<Double> averageDistance = Subsystems.visionOdometryUpdater.getTargetDistance();
        if (averageDistance.isPresent() && averageDistance.get() < 2.0) {
            Subsystems.poseManager.pushRequest(new PoseChangeRequest(Subsystems.visionOdometryUpdater.getEstimatedPose()));
        }

        // Obtain the current robot pose from the drivetrain
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d targetTranslation = targetPose.getTranslation();

        Rotation2d targetAngle = targetPose.getRotation();
        anglePID.setSetpoint(targetAngle.getDegrees());

        // Calculate the distance error and compute the translational speed command.
        double distanceError = currentTranslation.getDistance(targetTranslation);
        double driveSpeed = distancePID.calculate(distanceError);

        // Determine the drive angle (in field coordinates) from the current position to the target.
        double driveAngle = Math.atan2(
                targetTranslation.getY() - currentTranslation.getY(),
                targetTranslation.getX() - currentTranslation.getX()
        );

        // Decompose the commanded speed into x and y components.
        double vx = driveSpeed * Math.cos(driveAngle);
        double vy = driveSpeed * Math.sin(driveAngle);

        double baseRotation = anglePID.calculate(currentPose.getRotation().getDegrees());
        double rotationOutput = Degrees.of(baseRotation).times(DegreesPerSecond.of(360).in(RadiansPerSecond)).in(Radians);

        // Construct the chassis speeds command (assuming field-relative control)
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, rotationOutput);
        BSLogger.log("DriveToPoseCommand", "Chass   is Speeds: " + chassisSpeeds);
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(chassisSpeeds));
    }

    @Override
    public boolean isFinished() {
        // The command ends when both translation and rotation are within tolerance.
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
