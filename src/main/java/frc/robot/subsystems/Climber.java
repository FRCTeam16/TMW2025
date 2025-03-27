package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;

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
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        Slot0Configs slot0Configs = new Slot0Configs()
                .withKP(5.0);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(200)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(0);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(400)
                .withMotionMagicCruiseVelocity(100);

        climberConfiguration
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withSlot0(slot0Configs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

        climberMotor.getConfigurator().apply(climberConfiguration);
        climberMotor.setPosition(0.0);
        this.setDefaultCommand(this.defaultHoldPositionCommand());
    }

    @Override
    public void autoInit() {
        this.removeDefaultCommand();
    }

    @Override
    public void teleopInit() {
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

    public Command zeroClimberPosition() {
        return this.runOnce(() -> this.climberMotor.setPosition(0));
    }

    public Command defaultHoldPositionCommand() {
        return this.run(() -> Subsystems.climber.moveToPosition(Subsystems.climber.getPosition()));
    }

    public enum ClimberPosition {
        DOWN(0),
        PICKUP(75),
        UP(150),
        CLIMB(180);

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
            this.addRequirements(Subsystems.climber);
            if (ClimberPosition.PICKUP == position) {
                addRequirements(Subsystems.funnelSubsystem);
            }

            this.position = position;
        }

        @Override
        public void initialize() {
            if (ClimberPosition.PICKUP == position) {
                BSLogger.log("Climber", "Closing Latch");
                Subsystems.funnelSubsystem.closeLatch();
            } else if (ClimberPosition.CLIMB == position && Subsystems.funnelSubsystem.isLatchOpen()) {
                BSLogger.log("Climber", "Requested to climb but we think latch is open");
                return;
            }
            BSLogger.log("Climber", "Moving to position: " + position.name());
            Subsystems.climber.moveToPosition(this.position.position);
        }

        @Override
        public boolean isFinished() {
            return Subsystems.climber.isInPosition();
        }
    }

    public static class ClimberMoveToPositionNoWait extends Command {
        private final ClimberPosition position;

        public ClimberMoveToPositionNoWait(ClimberPosition position) {
            addRequirements(Subsystems.climber);
            if (ClimberPosition.PICKUP == position) {
                addRequirements(Subsystems.funnelSubsystem);
            }
            this.position = position;
        }

        @Override
        public void initialize() {
            if (ClimberPosition.PICKUP == position) {
                BSLogger.log("Climber", "Closing Latch");
                Subsystems.funnelSubsystem.closeLatch();
            }
            BSLogger.log("Climber", "Moving to position: " + position.name());
            Subsystems.climber.moveToPosition(this.position.position);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}
