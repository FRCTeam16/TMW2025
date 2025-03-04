package frc.robot.commands.path;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class DriveToPoseProfiledCommand extends Command {
    // Translation controller constants and tolerances
    private static final double kP_distance = 0.5;
    private static final double kI_distance = 0.0;
    private static final double kD_distance = 0.1;
    private static final double MAX_VELOCITY = 2.0;         // m/s (example)
    private static final double MAX_ACCELERATION = 1.0;       // m/s^2 (example)
    private static final double DISTANCE_TOLERANCE = 0.1;     // meters
    // Rotation controller constants and tolerances
    private static final double kP_angle = 0.1;
    private static final double kI_angle = 0.0;
    private static final double kD_angle = 0.02;
    private static final double MAX_ANGULAR_VELOCITY = Math.PI;          // rad/s (example)
    private static final double MAX_ANGULAR_ACCELERATION = Math.PI / 2;    // rad/s^2 (example)
    private static final double ANGLE_TOLERANCE = 0.05;         // radians
    private final CommandSwerveDrivetrain drivetrain = Subsystems.swerveSubsystem;
    private final Pose2d targetPose;
    private final ProfiledPIDController distancePID;
    private final ProfiledPIDController anglePID;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public DriveToPoseProfiledCommand(Pose2d targetPose) {
        this.targetPose = targetPose;

        // Set up the distance ProfiledPIDController with motion constraints.
        distancePID = new ProfiledPIDController(
                kP_distance, kI_distance, kD_distance,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
        );
        distancePID.setTolerance(DISTANCE_TOLERANCE);

        // Set up the angle ProfiledPIDController with motion constraints.
        anglePID = new ProfiledPIDController(
                kP_angle, kI_angle, kD_angle,
                new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION)
        );
        // For angles, enable continuous input over the full circle.
        anglePID.enableContinuousInput(-Math.PI, Math.PI);
        anglePID.setTolerance(ANGLE_TOLERANCE);

        SmartDashboard.putData("DriveToPoseProfiled/distancePID", distancePID);
        SmartDashboard.putData("DriveToPoseProfiled/angularPID", anglePID);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset the controllers with the current state.
        Pose2d initialPose = drivetrain.getState().Pose;
        double initialDistanceError = initialPose.getTranslation().getDistance(targetPose.getTranslation());
        distancePID.reset(initialDistanceError);
        anglePID.reset(initialPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d targetTranslation = targetPose.getTranslation();

        // Compute the distance error from the current position to the target.
        double distanceError = currentTranslation.getDistance(targetTranslation);
        // ProfiledPIDController calculates an output to drive the distance error to zero.
        double translationalOutput = distancePID.calculate(distanceError, 0.0);

        // Determine the field-relative angle from the current position toward the target.
        double driveAngle = Math.atan2(
                targetTranslation.getY() - currentTranslation.getY(),
                targetTranslation.getX() - currentTranslation.getX()
        );

        // Decompose the translational command into x and y components.
        double vx = translationalOutput * Math.cos(driveAngle);
        double vy = translationalOutput * Math.sin(driveAngle);

        // Calculate the rotational output to reach the target heading.
        double currentAngle = currentPose.getRotation().getRadians();
        double targetAngle = targetPose.getRotation().getRadians();
        double rotationOutput = anglePID.calculate(currentAngle, targetAngle);

        // Command the drivetrain using field-relative chassis speeds.
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx, vy, rotationOutput);
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(chassisSpeeds));
    }

    @Override
    public boolean isFinished() {
        // Finish when both controllers have reached their goals.
        return distancePID.atGoal() && anglePID.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot if the command is ended or interrupted.
        drivetrain.setControl(applyRobotSpeeds.withSpeeds(
                new ChassisSpeeds(0.0, 0.0, 0.0)));
    }
}
