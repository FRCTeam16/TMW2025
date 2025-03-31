package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;

import static edu.wpi.first.units.Units.Degrees;

public class RotateToAngleCommand extends Command {
    private final SwerveRequest.FieldCentric swerve = new SwerveRequest.FieldCentric().withVelocityX(0).withVelocityY(0);
    private final RotationController pid = new RotationController(0.016, 0, 0);
    private final Angle target;
    private int stopCounter = 0;

    public RotateToAngleCommand(Angle target) {
        this.target = target;
        addRequirements(Subsystems.swerveSubsystem);
    }

    @Override
    public void initialize() {
        stopCounter = 0;
        pid.reset();
        pid.setSetpoint(target.in(Degrees));
        pid.setTolerance(1.0);
    }

    @Override
    public void execute() {
        double currentDegrees = Subsystems.swerveSubsystem.getState().Pose.getRotation().getDegrees();
        double error = pid.calculate(currentDegrees, target.in(Degrees));
        AngularVelocity rotation = Constants.MaxAngularRate.times(error);
        Subsystems.swerveSubsystem.setControl(swerve.withRotationalRate(rotation));
    }

    @Override
    public boolean isFinished() {
        double current = Subsystems.swerveSubsystem.getState().Pose.getRotation().getDegrees();

        if (Math.abs(current - target.in(Degrees)) <= 1.0) {
            stopCounter++;
        }
        return stopCounter > 10;
    }
}
