package frc.robot.commands.dms;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DMSDataCollector;

import java.util.Arrays;

import static edu.wpi.first.units.Units.RadiansPerSecond;

class RunSteerTestCommand extends AbstractRunDMSMotorTestCommand {

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    public RunSteerTestCommand(DMSDataCollector dmsDataCollector) {
        super(dmsDataCollector);
    }

    @Override
    void startMotors() {
        Subsystems.swerveSubsystem.setControl(
                robotCentric
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(Constants.MaxAngularRate.times(1.0).in(RadiansPerSecond)));
    }

    @Override
    void stopMotors() {
        Subsystems.swerveSubsystem.setControl(
                robotCentric
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0));
    }

    @Override
    double[] getMotorCurrents() {
        return Arrays.stream(Subsystems.swerveSubsystem.getModules()).mapToDouble(module -> module.getSteerMotor().getStatorCurrent().getValueAsDouble()).toArray();
    }

    @Override
    double[] getMotorVelocities() {
        return Arrays.stream(Subsystems.swerveSubsystem.getModules()).mapToDouble(module -> module.getSteerMotor().getVelocity().getValueAsDouble()).toArray();
    }
}
