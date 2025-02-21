package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionTypes;
import frc.robot.util.PIDHelper;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.MaxAngularRate;

public class LimelightBasedAlignmentCommand extends Command {
    private final SwerveRequest.RobotCentric alignDrive = new SwerveRequest.RobotCentric();
    PIDController translationPID = new PIDController(0.1, 0, 0);
    PIDHelper translationPIDHelper = new PIDHelper("AlignmentTest");

    private final Angle targetAngle;    // offset angle to align
    private final String selectedLimelightName;
    private final TargetSide targetSide;

    public LimelightBasedAlignmentCommand(TargetSide targetSide) {
        this.addRequirements(Subsystems.swerveSubsystem);
        this.targetSide = targetSide;
        if (targetSide == TargetSide.LEFT) {
            this.targetAngle = Degrees.of(-3.0);
            this.selectedLimelightName = "limelight";
        } else if (targetSide == TargetSide.RIGHT) {
            this.targetAngle = Degrees.of(-3.0);
            this.selectedLimelightName = "limelight-right";
        } else {
            throw new IllegalArgumentException("Invalid target side");
        }

        this.translationPIDHelper.initialize(0.1, 0, 0, 0, 0, 0);
        this.translationPID.setTolerance(0.5);
    }

    @Override
    public void execute() {
        Optional<VisionTypes.TargetInfo> targetInfo = Subsystems.visionSubsystem.getTargetInfo();
        if (targetInfo.isEmpty()) {
            noopDrive();
            return;
        }

        // Lock angle and calculate translation speed
        AngularVelocity rotationRate = calculateRobotRotation();
        LinearVelocity yDriveSpeed = calculateRobotYTranslationSpeed();
        Subsystems.swerveSubsystem.setControl(
                alignDrive.withVelocityX(0)
                        .withRotationalRate(rotationRate)
                        .withVelocityY(yDriveSpeed));
    }

    private void noopDrive() {
        Subsystems.swerveSubsystem.setControl(
                alignDrive.withVelocityX(0)
                        .withRotationalRate(DegreesPerSecond.of(0))
                        .withVelocityY(0));
    }

    private AngularVelocity calculateRobotRotation() {
        final Angle robotFacingAngle = Degrees.of(0);    // TODO Need lookup based on AprilTag ID
        // Calculate angular velocity
        return DegreesPerSecond.of(
                        Subsystems.rotationController.calculate(
                                Subsystems.swerveSubsystem.getPigeon2().getYaw().getValueAsDouble(),
                                robotFacingAngle.in(Degrees)))
                .times(MaxAngularRate.in(RadiansPerSecond));
    }

    private LinearVelocity calculateRobotYTranslationSpeed() {
        translationPIDHelper.updatePIDController(translationPID);
        var errorDegrees = Degrees.of(LimelightHelpers.getTX(this.selectedLimelightName));
        LinearVelocity maxRobotSpeed = MetersPerSecond.of(0.5);
        var rawSpeed = translationPID.calculate(errorDegrees.in(Degrees), this.targetAngle.in(Degrees));
        LinearVelocity yDriveSpeed = MetersPerSecond.of(MathUtil.clamp(
                maxRobotSpeed.in(MetersPerSecond) * rawSpeed, -0.5, 0.5));

        System.out.println("YTRANS: " + errorDegrees + " | " + yDriveSpeed);
        if (this.translationPID.atSetpoint()) {
            yDriveSpeed = MetersPerSecond.of(0);
        }
        return yDriveSpeed;
    }

    public enum TargetSide {
        LEFT,
        RIGHT
    }
}
