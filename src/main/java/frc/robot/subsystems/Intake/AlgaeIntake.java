package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntake extends SubsystemBase {
    private TalonFX algaeMotor = new TalonFX(53);
    private NeutralOut brake = new NeutralOut();
    private DutyCycleOut dutyCycleOut = new DutyCycleOut(1);

    private double forwardSpeed = 0.3;
    private double backwardSpeed = -0.3;
    private double holdSpeed = 0.1;
    private double clampSpeed = 0.5;


    public AlgaeIntake() {
        algaeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic(){
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
        builder.setSmartDashboardType("AlgaeIntake");
        builder.addDoubleProperty("forwardSpeed", () -> forwardSpeed, this::setForwardSpeed);
        builder.addDoubleProperty("backwardSpeed", () -> backwardSpeed, this::setBackwardSpeed);
        builder.addDoubleProperty("clampSpeed", () -> clampSpeed, (v) -> clampSpeed = v);
        builder.addDoubleProperty("holdSpeed", () -> holdSpeed, this::setHoldSpeed);
    }

    public Command runForward(){
        return this.runOnce(() -> {
            algaeMotor.setControl(dutyCycleOut.withOutput(forwardSpeed));
        });
    }

    public Command runBackward(){
        return this.runOnce(() -> {
            algaeMotor.setControl(dutyCycleOut.withOutput(backwardSpeed));
        });
    }

    public Command hold() {
        return this.runOnce(() -> {
            algaeMotor.setControl(dutyCycleOut.withOutput(holdSpeed));
        });
    }

    public Command stop(){
        return this.runOnce(() -> {
            algaeMotor.setControl(brake);
        });
    }

}
