package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.BayouTroisStrategy;
import frc.robot.subsystems.RotationController;
import frc.robot.util.BSLogger;

import static edu.wpi.first.units.Units.*;

public class DriveRobotCentricCommand extends Command {
    private final RotationController rotationPID = new RotationController(0.016, 0, 0);
    private final Time toRun;
    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    private final SwerveRequest.RobotCentricFacingAngle robotCentricFacingRequest = new SwerveRequest.RobotCentricFacingAngle().withHeadingPID(Radians.convertFrom(rotationPID.getP(), Degrees), 0, 0);
    private final Timer timer = new Timer();
    private Pose2d lastPose;
    private LinearVelocity APPROACH_SPEED = MetersPerSecond.of(1.6);
    private Angle targetDegrees;

    private BayouTroisStrategy.StartingPosition startingPosition;

    public DriveRobotCentricCommand(Time runTime) {
        addRequirements(Subsystems.swerveSubsystem);
        toRun = runTime;
    }

    public DriveRobotCentricCommand withApproachSpeed(LinearVelocity velocity) {
        this.APPROACH_SPEED = velocity;
        return this;
    }

    public DriveRobotCentricCommand withCalcFacingAngle(BayouTroisStrategy.StartingPosition startingPosition) {
        this.startingPosition = startingPosition;
        return this;
    }

    @Override
    public void initialize() {
        Pose2d startPose = Subsystems.swerveSubsystem.getState().Pose;
        timer.restart();

        if (this.startingPosition != null) {
            // Calculate the target angle based on the starting position and AprilTag
            final int aprilTarget = switch (this.startingPosition) {
                case RED_LEFT -> 11;
                case RED_RIGHT -> 9;
                case BLUE_LEFT -> 20;
                case BLUE_RIGHT -> 22;
            };
            this.targetDegrees = Subsystems.aprilTagUtil.getTagPose2d(aprilTarget)
                    .map(pose2d -> pose2d.getRotation().getMeasure().plus(Degrees.of(180)))
                    .orElse(startPose.getRotation().getMeasure());
        } else {
            // By default, use the current robot pose for the target angle
            this.targetDegrees = startPose.getRotation().getMeasure();
        }
    }

    @Override
    public void execute() {
        Pose2d currentState = Subsystems.swerveSubsystem.getState().Pose;

        if (startingPosition == null) {
            double rotationError = rotationPID.calculate(currentState.getRotation().getDegrees(), targetDegrees.in(Degrees));
            BSLogger.log("DriveRobotCentricCommand", "Drive using robot pose, angle=" + targetDegrees);
            Subsystems.swerveSubsystem.setControl(
                    robotCentricRequest
                            .withVelocityX(APPROACH_SPEED)
                            .withVelocityY(0)
                            .withRotationalRate(Constants.MaxAngularRate.times(rotationError)));
        } else {
            BSLogger.log("DriveRobotCentricCommand", "Drive using facing angle, angle=" + targetDegrees);
            Subsystems.swerveSubsystem.setControl(
                    robotCentricFacingRequest
                            .withVelocityX(APPROACH_SPEED)
                            .withVelocityY(0)
                            .withTargetDirection(Rotation2d.fromDegrees(targetDegrees.in(Degrees)))
            );
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(toRun.in(Seconds));
    }
}
