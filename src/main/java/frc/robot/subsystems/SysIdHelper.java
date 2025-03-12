package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class SysIdHelper {
    private final SysIdRoutine m_sysIdRoutineTranslation;
    private final SysIdRoutine m_sysIdRoutineSteer;
    private final SysIdRoutine m_sysIdRoutineRotation;
    private SysIdRoutine m_sysIdRoutineToApply;

    public SysIdHelper(CommandSwerveDrivetrain drivetrain) {
        m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> drivetrain.setControl(new SwerveRequest.SysIdSwerveTranslation().withVolts(output)),
                null,
                drivetrain
            )
        );

        m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(7),
                null,
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> drivetrain.setControl(new SwerveRequest.SysIdSwerveSteerGains().withVolts(volts)),
                null,
                drivetrain
            )
        );

        m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(Math.PI / 6).per(Second),
                Volts.of(Math.PI),
                null,
                state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> {
                    drivetrain.setControl(new SwerveRequest.SysIdSwerveRotation().withRotationalRate(output.in(Volts)));
                    SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                },
                null,
                drivetrain
            )
        );

        m_sysIdRoutineToApply = m_sysIdRoutineTranslation;
    }

    public SysIdHelper withRoutineToApply(Routine routine) {
        this.m_sysIdRoutineToApply = switch(routine) {
            case Translation -> m_sysIdRoutineTranslation;
            case Steer -> m_sysIdRoutineSteer;
            case Rotation -> m_sysIdRoutineRotation;
        };
        return this;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public enum Routine {
        Translation, Steer, Rotation
    }
}
