package frc.robot.commands.amd;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystems;
import frc.robot.subsystems.amd.SwerveDataCollector;

import java.util.Arrays;

class RunSteerTestCommand extends AbstractRunDMSMotorTestCommand {
    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final VoltageOut steerRequest = new VoltageOut(0);
    private final CoastOut steerDriveRequest = new CoastOut();


    public RunSteerTestCommand(SwerveDataCollector swerveDataCollector) {
        super(swerveDataCollector);
    }

    @Override
    void startMotors() {
        SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = Subsystems.swerveSubsystem.getModules();
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : modules) {
            module.apply(steerDriveRequest, steerRequest.withOutput(12));
        }
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
