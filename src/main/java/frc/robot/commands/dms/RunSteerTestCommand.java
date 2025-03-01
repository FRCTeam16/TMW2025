package frc.robot.commands.dms;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.SwerveDataCollector;

import java.util.Arrays;

import static edu.wpi.first.units.Units.RadiansPerSecond;

class RunSteerTestCommand extends AbstractRunDMSMotorTestCommand {
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();


    public RunSteerTestCommand(SwerveDataCollector swerveDataCollector) {
        super(swerveDataCollector);
    }

    @Override
    void startMotors() {
        Subsystems.swerveSubsystem.setControl(
                applyRobotSpeeds.withSpeeds(
                        new ChassisSpeeds(0, 0, Constants.MaxAngularRate.in(RadiansPerSecond))));
    }

    @Override
    void stopMotors() {
        Subsystems.swerveSubsystem.setControl(
                applyRobotSpeeds.withSpeeds(new ChassisSpeeds()));
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
