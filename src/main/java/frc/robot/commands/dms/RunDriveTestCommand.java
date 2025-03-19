package frc.robot.commands.dms;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.SwerveDataCollector;
import frc.robot.util.BSLogger;

import java.util.Arrays;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
        return Arrays.stream(Subsystems.swerveSubsystem.getModules()).mapToDouble(module -> module.getDriveMotor().getStatorCurrent().getValueAsDouble()).toArray();
    }

    @Override
    double[] getMotorVelocities() {
        return Arrays.stream(Subsystems.swerveSubsystem.getModules())
                .mapToDouble(module -> module.getDriveMotor().getVelocity().getValueAsDouble())
                .toArray();
    }
}
