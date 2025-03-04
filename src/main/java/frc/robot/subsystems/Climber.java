package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;

public class Climber extends SubsystemBase implements Lifecycle {

    private final TalonFX climberMotor = new TalonFX(Robot.robotConfig.getCanID("climberMotor"));
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    private double openLoopMotorOutput = 0.5;
    private double currentSetpoint = 0;
    private double setpointVelocity = 10;

    public Climber() {
        TalonFXConfiguration climberConfiguration = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive);
        Slot0Configs slot0Configs = new Slot0Configs()
                .withKP(5.0);
        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(82)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(-70);
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(20)
                .withMotionMagicCruiseVelocity(20);

        climberConfiguration
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withSlot0(slot0Configs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

        climberMotor.getConfigurator().apply(climberConfiguration);
        this.setDefaultCommand(this.defaultHoldPositionCommand());
    }

    private double getPosition() {
        return climberMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("Climber");
        builder.addDoubleProperty("openLoopMotorOutput", () -> openLoopMotorOutput, (v) -> openLoopMotorOutput = v);
        builder.addDoubleProperty("currentSetpoint", () -> currentSetpoint, (v) -> currentSetpoint = v);
        builder.addDoubleProperty("position", this::getPosition, null);
        builder.addBooleanProperty("inPosition", this::isInPosition, null);
        builder.addDoubleProperty("setpointVelocity", () -> setpointVelocity, (v) -> setpointVelocity = v);
    }

    public boolean isInPosition() {
        return Math.abs(getPosition() - currentSetpoint) < 0.5;
    }

    private void runOpenLoop(double value) {
        climberMotor.setControl(dutyCycleOut.withOutput(value));
    }

    private void moveToPosition(double position) {
        this.currentSetpoint = position;
        climberMotor.setControl(
                motionMagicVoltage.withPosition(position));
//                positionVoltage.withPosition(position).withVelocity(setpointVelocity));
    }

    public Command openLoopUpDefault() {
        return this.run(() -> runOpenLoop(openLoopMotorOutput)).withName("Climber Default Open Loop Up");
    }

    public Command openLoopDownDefault() {
        return this.run(() -> runOpenLoop(-openLoopMotorOutput)).withName("Climber Default Open Loop Down");
    }

    public Command openLoopUp(double value) {
        return this.run(() -> runOpenLoop(value)).withName("Climber Open Loop Up");
    }

    public Command openLoopDown(double value) {
        return this.run(() -> runOpenLoop(-value)).withName("Climber Open Loop Up");
    }

    public Command defaultHoldPositionCommand() {
        return this.run(() -> {
            final double currentPosition = getPosition();
            moveToPosition(currentPosition);
        }).withName("Default Hold Climber Position");
    }


    public enum ClimberPosition {
        UP(0),
        CLIMB(-60), // -53.3
        DOWN(81),   // 100
        PICKUP(60); // 53.3

        private final double position;

        ClimberPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public static class ClimberMoveToPositionCommand extends Command {
        private final ClimberPosition position;

        public ClimberMoveToPositionCommand(ClimberPosition position) {
            addRequirements(Subsystems.climber);
            this.position = position;
        }

        @Override
        public void initialize() {
            Subsystems.climber.moveToPosition(this.position.position);
        }

        @Override
        public boolean isFinished() {
            return Subsystems.climber.isInPosition();
        }
    }
}
