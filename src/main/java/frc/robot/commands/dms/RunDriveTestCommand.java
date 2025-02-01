package frc.robot.commands.dms;

import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DMSDataCollector;

import java.util.Arrays;

import static edu.wpi.first.units.Units.MetersPerSecond;

class RunDriveTestCommand extends AbstractRunDMSMotorTestCommand {

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    public RunDriveTestCommand(DMSDataCollector dmsDataCollector) {
       super(dmsDataCollector);
    }

    @Override
    void startMotors() {
        Subsystems.swerveSubsystem.setControl(
        robotCentric
                .withVelocityX(MetersPerSecond.of(1.0).times(Constants.MaxSpeed.in(MetersPerSecond)))
                .withVelocityY(0)
                .withRotationalRate(0));
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
        return Arrays.stream(Subsystems.swerveSubsystem.getModules()).mapToDouble(module -> module.getDriveMotor().getStatorCurrent().getValueAsDouble()).toArray();
    }

    @Override
    double[] getMotorVelocities() {
        return Arrays.stream(Subsystems.swerveSubsystem.getModules()).mapToDouble(module -> module.getDriveMotor().getVelocity().getValueAsDouble()).toArray();
    }
}
