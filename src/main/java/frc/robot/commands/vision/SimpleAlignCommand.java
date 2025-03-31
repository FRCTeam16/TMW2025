package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.TranslationController;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.util.BSLogger;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class SimpleAlignCommand extends Command {
    private final boolean isLeft;
    private Angle targetRotation;
    private Distance targetDistance;

    private static final double TARGET_DISTANCE = 0.4;

    RotationController rotationController = Subsystems.rotationController;
    private PIDController alignController = Subsystems.alignTranslationController;
    private TranslationController distanceController = Subsystems.translationController;

    private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private SwerveRequest.Idle idle = new SwerveRequest.Idle();

    public SimpleAlignCommand(boolean isLeft) {
        this.isLeft = isLeft;
        this.targetDistance = Meters.of(isLeft ? TARGET_DISTANCE : TARGET_DISTANCE);
        this.addRequirements(Subsystems.swerveSubsystem);
    }


    @Override
    public void initialize() {
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        targetRotation = Subsystems.swerveSubsystem.getState().Pose.getRotation().getMeasure();
        if (hasTarget) {
            double aprilTarget = LimelightHelpers.getFiducialID("limelight");
            Optional<Pose2d> optTagPose = Subsystems.aprilTagUtil.getTagPose2d((int) aprilTarget);
            optTagPose.ifPresent(pose2d -> {
                targetRotation = pose2d.getRotation().getMeasure().plus(Degrees.of(180));
            });
        }

        rotationController.reset();
        rotationController.setTolerance(0.5);
        rotationController.setSetpoint(targetRotation.in(Degrees));

        distanceController.reset();
        distanceController.setTolerance(0.02);

        alignController.reset();
        alignController.setTolerance(0.25);
    }

    @Override
    public void execute() {
        System.out.println("SimpleAlignCommand");
        boolean hasTarget = LimelightHelpers.getTV("limelight");

        if (!hasTarget) {
            BSLogger.log("SimpleAlignCommand", "No visino target");
            Subsystems.swerveSubsystem.setControl(idle);
            return;
        }

        // Y Velocity (Vision alignment)
        double tx = LimelightHelpers.getTX("limelight");
        double target = calculateYTarget().in(Degrees);
        double yRrror = alignController.calculate(tx, target);
        yRrror = MathUtil.clamp(yRrror, -0.25, 0.25);
        LinearVelocity yspeed = Constants.MaxSpeed.times(yRrror);

        // X Velocity (Distance)
        double xError = 0;
        Optional<Double> distance = Subsystems.visionOdometryUpdater.getTargetDistance();
        LinearVelocity xspeed = MetersPerSecond.of(0);
        if (distance.isPresent()) {
            xError = -distanceController.calculate(distance.get(), this.targetDistance.in(Meters));
            xError = MathUtil.clamp(xError, -0.25, 0.25);
            xspeed = Constants.MaxSpeed.times(xError);
            System.out.println("DIST: " + distance.get() + " | Error: " + xError + " | Speed: " + xspeed);
        }

        // Rotation
        double currentDegrees = Subsystems.swerveSubsystem.getState().Pose.getRotation().getDegrees();
        double rotationError = rotationController.calculate(currentDegrees);
        AngularVelocity rotationSpeed = DegreesPerSecond.of(180).times(rotationError);

        LinearVelocity zero = MetersPerSecond.of(0);
        Subsystems.swerveSubsystem.setControl(
                robotCentric
                        .withVelocityX(xspeed)
                        .withVelocityY(yspeed)
                        .withRotationalRate(rotationSpeed)
        );
    }

    private Angle calculateYTarget() {
        Optional<Double> distance = Subsystems.visionOdometryUpdater.getTargetDistance();
        if (distance.isPresent()) {
            if (distance.get() > 0.6) {
                return Degrees.of((isLeft) ? 0 : 0);
            }
        }
//        return Degrees.of((isLeft) ? 22.5 : -23);
        return Degrees.of(0);
    }
}
