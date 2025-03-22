package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.TranslationController;
import frc.robot.subsystems.pose.PoseChangeRequest;
import frc.robot.subsystems.pose.UpdateTranslationFromVision;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.util.BSLogger;
import frc.robot.util.TimeExpiringValue;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class AlignDriveInCommand extends Command {
    private final Distance LIMELIGHT_OFFSET = Inches.of(12.279);

    private final AlignTarget alignTarget;
    private Angle targetRotation;
    private LinearVelocity approachSpeed = MetersPerSecond.of(1.25);

    RotationController rotationController = Subsystems.rotationController;
    private PIDController alignController = Subsystems.alignTranslationController;
    private TranslationController distanceController = Subsystems.translationController;

    private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private TimeExpiringValue<Pair<Pose2d, Angle>> lastVisionPose = new TimeExpiringValue<>(500);

    public enum AlignTarget {
        LEFT,
        RIGHT,
        CENTER
    }

    public AlignDriveInCommand(AlignTarget alignTarget) {
        this.alignTarget = alignTarget;
        this.addRequirements(Subsystems.swerveSubsystem);
    }

    public AlignDriveInCommand withApproachSpeed(LinearVelocity speed) {
        this.approachSpeed = speed;
        return this;
    }

    @Override
    public void initialize() {
        BSLogger.log("AlignDriveInCommand", "initialize");
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        targetRotation = Subsystems.swerveSubsystem.getState().Pose.getRotation().getMeasure();
        if (hasTarget) {
            double aprilTarget = LimelightHelpers.getFiducialID("limelight");
            Optional<Pose2d> optTagPose = Subsystems.aprilTagUtil.getTagPose2d((int) aprilTarget);
            optTagPose.ifPresent(pose2d -> {
                targetRotation = pose2d.getRotation().getMeasure().plus(Degrees.of(180));
                lastVisionPose.set(Pair.of(pose2d, Degrees.of(LimelightHelpers.getTX("limelight"))));
            });
            Subsystems.poseManager.pushRequest(new UpdateTranslationFromVision());
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
        boolean hasTarget = LimelightHelpers.getTV("limelight");

        final double rawtx;
        if (hasTarget) {
             rawtx = LimelightHelpers.getTX("limelight");
        } else {
            Optional<Pair<Pose2d, Angle>> optVisionTarget = lastVisionPose.get();
            if (optVisionTarget.isEmpty()) {
                Subsystems.swerveSubsystem.setControl(idle);
                return;
            } else {
                // Use last non-expired vision
                Pair<Pose2d, Angle> lastSeen = optVisionTarget.get();
                rawtx = lastSeen.getSecond().in(Degrees);
            }
        }

        Angle tx = Degrees.of(rawtx);
        double targetAngleRads = calculateYTargetDirect(tx).in(Radians);

        LinearVelocity xspeed = approachSpeed.times(Math.cos(targetAngleRads));
        LinearVelocity yspeed = approachSpeed.times(Math.sin(targetAngleRads));

        // Robot Rotation
        double currentDegrees = Subsystems.swerveSubsystem.getState().Pose.getRotation().getDegrees();
        double rotationError = rotationController.calculate(currentDegrees);
        AngularVelocity rotationSpeed = DegreesPerSecond.of(180).times(rotationError);

        Subsystems.swerveSubsystem.setControl(
                robotCentric
                        .withVelocityX(xspeed)
                        .withVelocityY(yspeed)
                        .withRotationalRate(rotationSpeed)
        );
    }

    /**
     * Calculates the target angle to directly drive towards a target based on robot position.
     * D is the hypothenuse of the triangle formed by the robot and the apriltag target.
     * d is the distance from the april tag to the target.
     * x is the distance from the robot to the target in the vertical axis.
     * y is the distance from the robot to the target in the horizontal axis.
     *
     * @param tx the angle to the target reported by the limelight
     * @return the angle to drive towards
     */
    private Angle calculateYTargetDirect(Angle tx) {
        Optional<Double> distance = Subsystems.visionOdometryUpdater.getTargetDistance();
        if (distance.isPresent()) {
            Angle angle = tx.times(-1);                         // invert incoming angle sign
            Distance D = Meters.of(distance.get());                      // distance to april tag
            Distance d = AlignTarget.CENTER == alignTarget ?
                    Inches.of(0) :                          // distance to center of ap
                    Inches.of(6.5);                         // distance to reef pole or center

            Distance x = D.times(Math.cos(angle.in(Radians)));             // R in reference system, the vertical vector
            x = x.minus(LIMELIGHT_OFFSET);                                 // limelight offset, take off the limelight offset within the robot
            Distance y = (D.times(Math.sin(angle.in(Radians)))).minus(d);  // r in reference system, the horizontal vector component

            // For right target add twice the small d distance
            if (AlignTarget.LEFT == alignTarget) {
                y = y.plus(d.times(2));
            }

            double rawResult = Math.atan2(y.in(Meters), x.in(Meters));
            Angle result = Radians.of(rawResult);

//            BSLogger.log("calculateYTarget", "tx=" + angle.in(Degrees));
//            BSLogger.log("calculateYTarget", "D =" + D);
//            BSLogger.log("calculateYTarget", "d =" + d);
//            BSLogger.log("calculateYTarget", "x =" + x);
//            BSLogger.log("calculateYTarget", "y =" + y);
//            BSLogger.log("calculateYTarget", "R =" + result.in(Degrees));
            return result;
        }
        // Default if no distance read
        return Degrees.of((AlignTarget.LEFT == alignTarget) ? 22.5 : -23);
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.swerveSubsystem.setControl(brake);

        boolean hasTarget = LimelightHelpers.getTV("limelight");
        if (hasTarget) {
            Subsystems.poseManager.pushRequest(new UpdateTranslationFromVision());
        } else {
            Pair<Pose2d, Angle> lastPose = lastVisionPose.getLastIgnoringExpiration();
            if (lastPose != null) {
                Subsystems.poseManager.pushRequest(new PoseChangeRequest(lastPose.getFirst()));
            }
        }
    }

    @Override
    public boolean isFinished() {
        Optional<Double> distance = Subsystems.visionOdometryUpdater.getTargetDistance();
        return distance.map(aDouble -> aDouble < 0.02).orElse(true);
    }
}

