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
import frc.robot.subsystems.RotationController;
import frc.robot.util.BSLogger;

import static edu.wpi.first.units.Units.*;

public class DriveRobotCentricCommand extends Command {
    private SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    private final RotationController rotationPID = new RotationController(0.016, 0, 0);
    private Timer timer = new Timer();
    private Pose2d startPose;
    private Pose2d lastPose;
    private final Time toRun;
    private final LinearVelocity APPROACH_SPEED =  MetersPerSecond.of(1.6);

    public DriveRobotCentricCommand(Time runTime) {
        addRequirements(Subsystems.swerveSubsystem);
        toRun = runTime;
    }

    @Override
    public void initialize() {
        this.startPose = Subsystems.swerveSubsystem.getState().Pose;
        timer.restart();
    }

    @Override
    public void execute() {
        Pose2d currentState = Subsystems.swerveSubsystem.getState().Pose;
        double targetDegrees = startPose.getRotation().getDegrees();
        double rotationError = rotationPID.calculate(currentState.getRotation().getDegrees(), targetDegrees);
        BSLogger.log("DEBUG", "Target angle is: " + targetDegrees);
        Subsystems.swerveSubsystem.setControl(
                robotCentricRequest
                        .withVelocityX(APPROACH_SPEED)
                        .withVelocityY(0)
                        .withRotationalRate(Constants.MaxAngularRate.times(rotationError)));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(toRun.in(Seconds));
    }
}
