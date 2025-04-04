package frc.robot.commands.amd;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.amd.SwerveDataCollector;
import frc.robot.util.BSLogger;

class RunDriveTestCommand extends AbstractRunDMSMotorTestCommand {

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public RunDriveTestCommand(SwerveDataCollector swerveDataCollector) {
        super(swerveDataCollector);
    }

    @Override
    void startMotors() {
        BSLogger.log("DMS RunDriveTestCommand", "startMotors");
        Subsystems.swerveSubsystem.setControl(
                applyRobotSpeeds.withSpeeds(
                        new ChassisSpeeds(Constants.MaxSpeed.in(MetersPerSecond), 0, 0)));
    }

    @Override
    void stopMotors() {
        BSLogger.log("DMS RunDriveTestCommand", "stopMotors");

        Subsystems.swerveSubsystem.setControl(
                applyRobotSpeeds.withSpeeds(new ChassisSpeeds()));
    }

    @Override
    double[] getMotorCurrents() {
        double[] data = new double[4];
        for (int i=0;i<4;i++) {
            var module = Subsystems.swerveSubsystem.getModule(i);
            data[i] = module.getDriveMotor().getStatorCurrent().getValueAsDouble();
        }
        return data;
    }

    @Override
    double[] getMotorVelocities() {
       double[] data = new double[4];
        for (int i=0;i<4;i++) {
            var module = Subsystems.swerveSubsystem.getModule(i);
            data[i] = module.getDriveMotor().getVelocity().getValueAsDouble();
        }
        return data;
    }

    @Override
    void report() {
        this.swerveDataCollector.report(true);
    }
}
