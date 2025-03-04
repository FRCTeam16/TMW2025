package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionTypes;
import frc.robot.util.BSLogger;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

@Deprecated
public class LimelightBasedAlignmentCommand extends Command {
    private final SwerveRequest.RobotCentric alignDrive = new SwerveRequest.RobotCentric();
    private final Angle targetAngle;    // offset angle to align
    private final String selectedLimelightName;
    RotationController rotationController = Subsystems.rotationController;
    PIDController translationPID = Subsystems.alignTranslationController;

    Optional<VisionTypes.TargetInfo> targetInfo = Optional.empty();


    public LimelightBasedAlignmentCommand(boolean isLeft) {
        this.addRequirements(Subsystems.swerveSubsystem);
        if (isLeft) {
            this.targetAngle = Degrees.of(20.0);
            this.selectedLimelightName = "limelight";
        } else {
            this.targetAngle = Degrees.of(-22.0);
            this.selectedLimelightName = "limelight";
        }
    }

    @Override
    public void initialize() {
        translationPID.reset();
        rotationController.reset();
        translationPID.setSetpoint(this.targetAngle.in(Degrees));
        translationPID.setTolerance(0.5);
        this.rotationController.setTolerance(1);
    }

    @Override
    public void execute() {
        System.out.println("*************************");
        System.out.println("*************************");
        System.out.println("*************************");

        targetInfo = Subsystems.visionSubsystem.getTargetInfo();

        if (targetInfo.isEmpty()) {
            BSLogger.log("LimelightAlign", "No target");
            noopDrive();
            return;
        }

        // Lock angle and calculate translation speed
        AngularVelocity rotationRate = calculateRobotRotation(targetInfo.get());
        LinearVelocity xDriveSpeed = getVelocityX();
        LinearVelocity yDriveSpeed = ManualCalculateRobotYTranslationSpeed();

        xDriveSpeed = MetersPerSecond.of(0);

        SmartDashboard.putNumber("LimelightAlign/xSpeed", xDriveSpeed.in(MetersPerSecond));
        SmartDashboard.putNumber("LimelightAlign/ySpeed", yDriveSpeed.in(MetersPerSecond));
        SmartDashboard.putNumber("LimelightAlign/rSpeed", rotationRate.in(DegreesPerSecond));
        SmartDashboard.putNumber("LimelightAlign/tx", targetInfo.get().xOffset());
        SmartDashboard.putNumber("LimelightAlign/setpoint", translationPID.getSetpoint());


        Subsystems.swerveSubsystem.setControl(
                alignDrive.withVelocityX(xDriveSpeed)
                        .withRotationalRate(rotationRate)
                        .withVelocityY(yDriveSpeed));


        System.out.println("*************************");
        System.out.println("*************************");
        System.out.println("*************************");

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
        final Angle robotFacingAngle = aprilTagAngle; // .plus(Degrees.of(180));
        // Calculate angular velocity
        return DegreesPerSecond.of(
                        rotationController.calculate(
                                Subsystems.swerveSubsystem.getState().Pose.getRotation().getDegrees(),
                                robotFacingAngle.in(Degrees)))
                .times(DegreesPerSecond.of(180).in(RadiansPerSecond));
    }

    private LinearVelocity getVelocityX() {
        double mps = RobotContainer.getInstance().getSwerveSupplier().supplyX().in(MetersPerSecond);
        double clamped_mps = MathUtil.clamp(mps, -0.5, 0.5);
        return MetersPerSecond.of(clamped_mps);
    }

    private LinearVelocity calculateRobotYTranslationSpeed() {
        if (targetInfo.isEmpty()) {
            return MetersPerSecond.of(0);
        }
//        Angle errorDegrees = Degrees.of(LimelightHelpers.getTX(this.selectedLimelightName));
        LinearVelocity maxRobotSpeed = MetersPerSecond.of(0.5);
        double rawSpeed = translationPID.calculate(targetInfo.get().xOffset());
        rawSpeed = MathUtil.clamp(rawSpeed, -0.25, 0.25);
        LinearVelocity yDriveSpeed = Constants.MaxSpeed.times(rawSpeed);

        if (this.translationPID.atSetpoint()) {
            yDriveSpeed = MetersPerSecond.of(0);
        }
        return yDriveSpeed;
    }

    private LinearVelocity ManualCalculateRobotYTranslationSpeed() {
        boolean hasTarget = LimelightHelpers.getTV("limelight");
        double tx = LimelightHelpers.getTX("limelight");

        System.out.println("@@@@@ TX: " + tx);

        LinearVelocity yDriveSpeed = MetersPerSecond.of(0);
        if (hasTarget) {
            double error = translationPID.calculate(tx, this.targetAngle.in(Degrees));
            error = MathUtil.clamp(error, -0.15, 0.15);
            yDriveSpeed = Constants.MaxSpeed.times(error);
        }
        if (translationPID.atSetpoint()) {
            yDriveSpeed = MetersPerSecond.of(0);
        }
        return yDriveSpeed;
    }



}
