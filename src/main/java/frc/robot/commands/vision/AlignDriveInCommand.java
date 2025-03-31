package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final Distance DEFAULT_TARGET_DISTANCE = Meters.of(0.375);
    private final Distance LIMELIGHT_OFFSET = Inches.of(12.279);

    private final AlignTarget alignTarget;
    private Angle targetRotation;
    private LinearVelocity approachSpeed = MetersPerSecond.of(1.25);
    private Distance targetDistance = DEFAULT_TARGET_DISTANCE;
    private boolean useTargetDistanceForApproachSpeed = false;

    RotationController rotationController = Subsystems.rotationController;
    private final PIDController alignController = Subsystems.alignTranslationController;
    private final TranslationController distanceController = Subsystems.translationController;

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final String limelightName;
    private final TimeExpiringValue<Pair<Pose2d, Angle>> lastVisionPose = new TimeExpiringValue<>(500);

    public enum AlignTarget {
        LEFT,
        RIGHT,
        CENTER
    }

    static AlignTelemetry alignTelemetry;

    public AlignDriveInCommand(AlignTarget alignTarget) {
        this.alignTarget = alignTarget;
        this.addRequirements(Subsystems.swerveSubsystem);
        this.limelightName = "limelight";
        SmartDashboard.putNumber("AlignDrive/Approach", 1.25);

        if (alignTelemetry == null) {
            alignTelemetry = new AlignTelemetry();
        }
    }

    public AlignDriveInCommand withApproachSpeed(LinearVelocity speed) {
        this.approachSpeed = speed;
        return this;
    }

    public AlignDriveInCommand withTargetDistance(Distance distance) {
        this.targetDistance = distance;
        return this;
    }

    @Override
    public void initialize() {
        alignTelemetry.activeLog.append(true);
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        targetRotation = Subsystems.swerveSubsystem.getState().Pose.getRotation().getMeasure();
        BSLogger.log("AlignDriveInCommand", "initialize: hasTarget?" + hasTarget);
        if (hasTarget) {
            updateVisionInfo();
        }

        rotationController.reset();
        rotationController.setTolerance(0.5);
        rotationController.setSetpoint(targetRotation.in(Degrees));

        distanceController.reset();
        distanceController.setTolerance(0.02);
        distanceController.setSetpoint(targetDistance.in(Meters));

        alignController.reset();
        alignController.setTolerance(0.25);
    }

    private void updateVisionInfo() {
        double aprilTarget = LimelightHelpers.getFiducialID(limelightName);
        Optional<Pose2d> optTagPose = Subsystems.aprilTagUtil.getTagPose2d((int) aprilTarget);
        optTagPose.ifPresent(pose2d -> {
            targetRotation = pose2d.getRotation().getMeasure().plus(Degrees.of(180));
            lastVisionPose.set(Pair.of(pose2d, Degrees.of(LimelightHelpers.getTX(limelightName))));
            // Reset our pose based on vision
            Subsystems.poseManager.pushRequest(new UpdateTranslationFromVision());
//            BSLogger.log("AlignDriveInCommand", "updating pose based on april tag [" + aprilTarget + "] " + pose2d);
        });
    }

    @Override
    public void execute() {
        final double rawtx;

        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        if (hasTarget) {
            updateVisionInfo();
            rawtx = LimelightHelpers.getTX(limelightName);
        } else {
            // No target detected, see if we can use previous vision info
            Optional<Pair<Pose2d, Angle>> optVisionTarget = lastVisionPose.get();
            if (optVisionTarget.isEmpty()) {
                BSLogger.log("AlignDriveInCommand", "execute has no valid vision, idling");
                Subsystems.swerveSubsystem.setControl(idle);
                return;
            } else {
                // Use last non-expired vision
                BSLogger.log("AlignDriveInCommand", "execute using cached vision target: " + optVisionTarget);
                Pair<Pose2d, Angle> lastSeen = optVisionTarget.get();
                rawtx = lastSeen.getSecond().in(Degrees);
            }
        }

        Angle tx = Degrees.of(rawtx);
        alignTelemetry.visionOffsetLog.append(rawtx);
        double headingRadians = calculateYTargetDirect(tx).in(Radians);

        //
        // Calculate speeds
        //
        approachSpeed = MetersPerSecond.of(SmartDashboard.getNumber("AlignDrive/Approach", 1.25));

        // Approach speed is the speed we want to drive towards the target
        LinearVelocity xspeed = approachSpeed.times(Math.cos(headingRadians));
        if (useTargetDistanceForApproachSpeed) {
            Optional<Double> seenDistance = Subsystems.visionOdometryUpdater.getTargetDistance();
            xspeed = seenDistance.map(seenDistanceM -> calculateXSpeedFromDistance(Meters.of(seenDistanceM)))
                    .orElseGet(() -> approachSpeed.times(Math.cos(headingRadians)));
        }

        // Y speed is the speed we want to drive horizontally towards the target point
        LinearVelocity yspeed = approachSpeed.times(Math.sin(headingRadians));

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

        alignTelemetry.periodic(Radians.of(headingRadians), xspeed, yspeed, rotationSpeed);
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

    private LinearVelocity calculateXSpeedFromDistance(Distance distance) {
        double error = distanceController.calculate(distance.in(Meters), targetDistance.in(Meters));
        return approachSpeed.times(-error);
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.swerveSubsystem.setControl(brake);

        if (LimelightHelpers.getTV(limelightName)) {
            Subsystems.poseManager.pushRequest(new UpdateTranslationFromVision());
        } else {
            Pair<Pose2d, Angle> lastPose = lastVisionPose.getLastIgnoringExpiration();
            if (lastPose != null) {
                Subsystems.poseManager.pushRequest(new PoseChangeRequest(lastPose.getFirst()));
            }
        }
        alignTelemetry.activeLog.append(false);
    }

    @Override
    public boolean isFinished() {
        boolean finishedRotation = rotationController.atSetpoint();
        boolean isAuto = DriverStation.isAutonomous();
        if (lastVisionPose.get().isEmpty() && isAuto) {
            BSLogger.log("AlignDriveInCommand", "Finishing command because no vision target cached");
            return true;
        }
        Optional<Double> distance = Subsystems.visionOdometryUpdater.getTargetDistance();
        if (distance.isPresent()) {
            boolean metDistance = distance.map(aDouble -> aDouble < targetDistance.in(Meters)).orElse(false);
            if (metDistance) {
                BSLogger.log("AlignDriveInCommand", "Finishing because we are within distance threshold");
                return true;
            };
            alignTelemetry.visionDistanceLog.append(distance.get());
        }

        return false;
    }

    class AlignTelemetry {
        final BooleanLogEntry activeLog;
        final DoubleLogEntry distanceLog;
        final DoubleLogEntry rotationLog;
        final DoubleLogEntry alignLog;
        final DoubleLogEntry visionOffsetLog;
        final DoubleLogEntry visionDistanceLog;

        final DoubleLogEntry xspeedLog;
        final DoubleLogEntry yspeedLog;
        final DoubleLogEntry rotSpeedLog;
        final DoubleLogEntry headingLog;

        AlignTelemetry() {
            DataLog log = DataLogManager.getLog();
            activeLog = new BooleanLogEntry(log, "Telemetry/AlignDrive/Active");
            headingLog = new DoubleLogEntry(log, "Telemetry/AlignDrive/Heading");
            distanceLog = new DoubleLogEntry(log, "Telemetry/AlignDrive/DistanceError");
            rotationLog = new DoubleLogEntry(log, "Telemetry/AlignDrive/RotationError");
            alignLog = new DoubleLogEntry(log, "Telemetry/AlignDrive/AlignError");
            visionOffsetLog = new DoubleLogEntry(log, "Telemetry/AlignDrive/VisionOffset");
            visionDistanceLog = new DoubleLogEntry(log, "Telemetry/AlignDrive/VisionDistance");

            xspeedLog = new DoubleLogEntry(log, "Telemetry/AlignDrive/XSpeed");
            yspeedLog = new DoubleLogEntry(log, "Telemetry/AlignDrive/YSpeed");
            rotSpeedLog = new DoubleLogEntry(log, "Telemetry/AlignDrive/RotSpeed");
        }

        void periodic(Angle angle, LinearVelocity xspeed, LinearVelocity yspeed, AngularVelocity rotationSpeed) {
            headingLog.append(angle.in(Degrees));
            xspeedLog.append(xspeed.in(MetersPerSecond));
            yspeedLog.append(yspeed.in(MetersPerSecond));
            rotSpeedLog.append(rotationSpeed.in(DegreesPerSecond));

            distanceLog.append(distanceController.getError());
            rotationLog.append(rotationController.getError());
            alignLog.append(alignController.getError());
        }
    }
}

