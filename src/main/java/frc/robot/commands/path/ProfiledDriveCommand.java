package frc.robot.commands.path;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.pose.UpdateTranslationFromVision;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.util.BSLogger;

public class ProfiledDriveCommand extends Command {

    // PID controllers for translation and rotation
    private final ProfiledPIDController distancePID;
    private final RotationController rotationPID = Subsystems.rotationController;
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

    private final Pose2d targetPose;
    private State initialState = new State(0, 0);
    private State finalState = new State(0, 0);

    private Distance tolerance = Meters.of(0.02);

    public ProfiledDriveCommand(Pose2d targetPose) {
        this.targetPose = targetPose;
        distancePID = new ProfiledPIDController(
                Constants.AutoConstants.kDriveP,
                Constants.AutoConstants.kDriveI,
                Constants.AutoConstants.kDriveD,
                new TrapezoidProfile.Constraints(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        addRequirements(Subsystems.swerveSubsystem);
    }

    public ProfiledDriveCommand withStartState(State initialState) {
        this.initialState = initialState;
        return this;
    }

    public ProfiledDriveCommand withFinalState(State finalState) {
        this.finalState = finalState;
        return this;
    }

    public ProfiledDriveCommand withTolerance(Distance tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    @Override
    public void initialize() {
        distancePID.reset(initialState);
        distancePID.setTolerance(tolerance.in(Meters));

        Pose2d currentPose = Subsystems.swerveSubsystem.getState().Pose;
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        distancePID.setGoal(new TrapezoidProfile.State(distance, finalState.velocity));
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        Optional<Double> visionDistance = Subsystems.visionOdometryUpdater.getTargetDistance();
        if (hasTarget && visionDistance.isPresent() && visionDistance.get() < 2.0 ) {
            Subsystems.poseManager.pushRequest(new UpdateTranslationFromVision());
        }

        // Get current pose
        Pose2d currentPose = Subsystems.swerveSubsystem.getState().Pose;

        // Calculate distance and angle to target
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        double targetAngle = targetPose.getRotation().getRadians();
        double currentAngle = currentPose.getRotation().getRadians();

        // Calculate the heading to target using atan2
        double headingToTarget = Math.atan2(
                targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX());

        // Use the ProfiledPIDControllers to calculate outputs
        double driveOutput = distancePID.calculate(distance, finalState);

        AngularVelocity rotationOutput = Constants.MaxAngularRate.times(rotationPID.calculate(currentAngle, targetAngle));


        BSLogger.log("ProfiledDriveCommand", "Distance; " + distance);
        //  +
        //         " " + driveOutput +
        //         " " + rotationOutput +
        //         " " + Radians.of(headingToTarget).in(Degrees));

        // Subsystems.swerveSubsystem.setControl(
        //         applyRobotSpeeds.withSpeeds(
        //                 ChassisSpeeds.fromFieldRelativeSpeeds(
        //                         driveOutput * Math.cos(headingToTarget),
        //                         driveOutput * Math.sin(headingToTarget),
        //                         rotationOutput.in(RadiansPerSecond),
        //                         currentPose.getRotation())));
        
        Subsystems.swerveSubsystem.setControl(
            fieldCentric.withVelocityX(driveOutput * Math.cos(headingToTarget))
            .withVelocityY(driveOutput * Math.sin(headingToTarget))
            .withRotationalRate(rotationOutput));
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = Subsystems.swerveSubsystem.getState().Pose;
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        return Math.abs(distance) < this.tolerance.in(Meters);
    }
}
