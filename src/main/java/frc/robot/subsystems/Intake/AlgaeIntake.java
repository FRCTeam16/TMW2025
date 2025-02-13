package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lifecycle;

import static edu.wpi.first.units.Units.Degrees;

public class AlgaeIntake extends SubsystemBase implements Lifecycle {
    private final TalonFX algaeIntakeMotor = new TalonFX(Robot.robotConfig.getCanID("algaeIntakeMotor"));
    private final NeutralOut brake = new NeutralOut();
    private final DutyCycleOut intakeDutyCycleOut = new DutyCycleOut(1);
    // TODO: Investigate StaticBrake

    private double forwardSpeed = 0.3;
    private double backwardSpeed = -0.3;
    private double holdSpeed = 0.1;
    private double clampSpeed = 0.5;    // TODO: we can remove clamping speed control once more testing is done

    public AlgaeIntake() {
        TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration();
        algaeIntakeMotor.getConfigurator().apply(intakeConfiguration);
        algaeIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
        this.setDefaultCommand(this.stopCommand());
    }

    public void setForwardSpeed(double speed) {
        this.forwardSpeed = clampSpeed(speed);
    }

    public void setBackwardSpeed(double speed) {
        this.backwardSpeed = clampSpeed(speed);
    }

    public void setHoldSpeed(double speed) {
        this.holdSpeed = clampSpeed(speed);
    }

    private double clampSpeed(double speed) {
        return MathUtil.clamp(speed, -clampSpeed, clampSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("AlgaeIntake");
        builder.addDoubleProperty("forwardSpeed", () -> forwardSpeed, this::setForwardSpeed);
        builder.addDoubleProperty("backwardSpeed", () -> backwardSpeed, this::setBackwardSpeed);
        builder.addDoubleProperty("clampSpeed", () -> clampSpeed, (v) -> clampSpeed = v);
        builder.addDoubleProperty("holdSpeed", () -> holdSpeed, this::setHoldSpeed);
    }

    public Command intakeCommand() {
        return this.runOnce(() -> {
            algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(forwardSpeed));
        });
    }

    public Command ejectCommand() {
        return this.runOnce(() -> {
            algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(backwardSpeed));
        });
    }

    public Command holdAlgaeCommand() {
        return this.runOnce(() -> {
            algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(holdSpeed));
        });
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            algaeIntakeMotor.setControl(brake);
        }).withName("Algae Stop");
    }

}
