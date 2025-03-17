package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
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
import frc.robot.subsystems.pose.UpdateTranslationFromVision;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.util.BSLogger;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class AlignDriveInCommandAlgea extends Command {
    private static final double TARGET_DISTANCE = 0.4;

    private final Distance LIMELIGHT_OFFSET = Inches.of(12.279);


    RotationController rotationController = Subsystems.rotationController;
    private Distance targetDistance;
    private Angle targetRotation;
    private PIDController alignController = Subsystems.alignTranslationController;
    private TranslationController distanceController = Subsystems.translationController;

    private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    
        public AlignDriveInCommandAlgea() {
        this.targetDistance = Meters.of(TARGET_DISTANCE);
        this.addRequirements(Subsystems.swerveSubsystem);
    }

    public AlignDriveInCommandAlgea withTargetDistance(Distance distance) {
        this.targetDistance = distance;
        return this;
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
        BSLogger.log("AlignDriveInCommandAlgea", "execute");
        boolean hasTarget = LimelightHelpers.getTV("limelight");

        if (!hasTarget) {
            Subsystems.swerveSubsystem.setControl(idle);
            return;
        }

        double rawtx = LimelightHelpers.getTX("limelight");
        Angle tx = Degrees.of(rawtx);
        double targetAngleRads = calculateYTargetDirect(tx).in(Radians);

        LinearVelocity speed = MetersPerSecond.of(1.5);
        LinearVelocity xspeed = speed.times(Math.cos(targetAngleRads));
        LinearVelocity yspeed = speed.times(Math.sin(targetAngleRads));

        // Robot Rotation
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

    private Angle calculateYTargetDirect(Angle tx) {
        Optional<Double> distance = Subsystems.visionOdometryUpdater.getTargetDistance();
        if (distance.isPresent()) {
            Angle angle = tx.times(-1);                         // invert incoming angle sign
            Distance D = Meters.of(distance.get());                      // distance to april tag
            Distance d = Inches.of(6.5);                       // distance to reef pole

            Distance x = D.times(Math.cos(angle.in(Radians)));             // R in reference system
            x = x.minus(LIMELIGHT_OFFSET);                                 // limelight offset
            Distance y = (D.times(Math.sin(angle.in(Radians)))).minus(d);  // r in reference system

            // For right target add
            if (true) {
                y = y.plus(d.times(1.5));
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
        return Degrees.of(-1);
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.swerveSubsystem.setControl(brake);
    }

    @Override
    public boolean isFinished() {
        Optional<Double> distance = Subsystems.visionOdometryUpdater.getTargetDistance();
        if (distance.isPresent()) {
            return distance.get() < 0.02;
        }
        return true;
    }
}

