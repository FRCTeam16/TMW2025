package frc.robot.commands.amd;

import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems;
import frc.robot.subsystems.amd.SwerveDataCollector;

class RunSteerTestCommand extends AbstractRunDMSMotorTestCommand {
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    SwerveRequest.SysIdSwerveSteerGains spinControl = new SwerveRequest.SysIdSwerveSteerGains().withVolts(7);


    public RunSteerTestCommand(SwerveDataCollector swerveDataCollector) {
        super(swerveDataCollector);
    }

    @Override
    void startMotors() {
        Subsystems.swerveSubsystem.setControl(spinControl);
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

    @Override
    void report() {
        this.swerveDataCollector.report(false);
    }
}
