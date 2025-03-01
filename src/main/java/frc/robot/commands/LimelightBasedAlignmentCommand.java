package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionTypes;
import frc.robot.util.BSLogger;
import frc.robot.util.PIDHelper;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.MaxAngularRate;

public class LimelightBasedAlignmentCommand extends Command {
    private final SwerveRequest.RobotCentric alignDrive = new SwerveRequest.RobotCentric();
    private final Angle targetAngle;    // offset angle to align
    private final String selectedLimelightName;
    PIDController translationPID = Subsystems.translationController;


    public LimelightBasedAlignmentCommand(boolean isLeft) {
        this.addRequirements(Subsystems.swerveSubsystem);
        if (isLeft) {
            this.targetAngle = Degrees.of(18.0);
            this.selectedLimelightName = "limelight-left";
        } else {
            this.targetAngle = Degrees.of(-25.0);
            this.selectedLimelightName = "limelight-right";
        }
    }

    @Override
    public void initialize() {
        this.translationPID.setTolerance(0.5);
    }

    @Override
    public void execute() {
        Optional<VisionTypes.TargetInfo> targetInfo = Subsystems.visionSubsystem.getTargetInfo();
        if (targetInfo.isEmpty()) {
            BSLogger.log("LimelightAlign", "No target");
            noopDrive();
            return;
        }

        // Lock angle and calculate translation speed
        AngularVelocity rotationRate = calculateRobotRotation(targetInfo.get());
        LinearVelocity xDriveSpeed = getVelocityX();
        LinearVelocity yDriveSpeed = calculateRobotYTranslationSpeed();
        Subsystems.swerveSubsystem.setControl(
                alignDrive.withVelocityX(xDriveSpeed)
                        .withRotationalRate(rotationRate)
                        .withVelocityY(yDriveSpeed));
    }

    private void noopDrive() {
        Subsystems.swerveSubsystem.setControl(
                alignDrive.withVelocityX(0)
                        .withRotationalRate(DegreesPerSecond.of(0))
                        .withVelocityY(0));
    }

    private AngularVelocity calculateRobotRotation(VisionTypes.TargetInfo targetInfo) {
        int aprilTag = targetInfo.aprilTagID();
        Optional<Pose2d> pose2d = Subsystems.aprilTagUtil.getTagPose2d(aprilTag);
        if (pose2d.isEmpty()) {
            return DegreesPerSecond.of(0);
        }
        Angle aprilTagAngle = pose2d.get().getRotation().getMeasure();
        final Angle robotFacingAngle = aprilTagAngle.plus(Degrees.of(180));
        // Calculate angular velocity
        return DegreesPerSecond.of(
                        Subsystems.rotationController.calculate(
                                Subsystems.swerveSubsystem.getPigeon2().getYaw().getValueAsDouble(),
                                robotFacingAngle.in(Degrees)))
                .times(MaxAngularRate.in(RadiansPerSecond));
    }

    private LinearVelocity getVelocityX() {
        double mps = RobotContainer.getInstance().getSwerveSupplier().supplyX().in(MetersPerSecond);
        double clamped_mps = MathUtil.clamp(mps, -0.5, 0.5);
        return MetersPerSecond.of(clamped_mps);
    }

    private LinearVelocity calculateRobotYTranslationSpeed() {
        var errorDegrees = Degrees.of(LimelightHelpers.getTX(this.selectedLimelightName));
        LinearVelocity maxRobotSpeed = MetersPerSecond.of(0.5);
        var rawSpeed = -translationPID.calculate(errorDegrees.in(Degrees), this.targetAngle.in(Degrees));
        LinearVelocity yDriveSpeed = MetersPerSecond.of(MathUtil.clamp(
                maxRobotSpeed.in(MetersPerSecond) * rawSpeed, -0.5, 0.5));

        BSLogger.log("BasicVision", "YTRANS: " + errorDegrees + " | " + yDriveSpeed);
        if (this.translationPID.atSetpoint()) {
            yDriveSpeed = MetersPerSecond.of(0);
        }
        return yDriveSpeed;
    }
}
