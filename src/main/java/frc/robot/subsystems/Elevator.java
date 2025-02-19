package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;

import java.util.function.Supplier;

public class Elevator extends SubsystemBase implements Lifecycle {
    public static final double GRAVITY_VOLTS = -0.5;
    public static final double ELEVATOR_POSITION_THRESHOLD = 0.1;
    private final TalonFX left = new TalonFX(Robot.robotConfig.getCanID("elevatorLeftMotor"));
    private final TalonFX right = new TalonFX(Robot.robotConfig.getCanID("elevatorRightMotor"));

    private final NeutralOut neutralOut = new NeutralOut();
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicV = new MotionMagicVoltage(0).withFeedForward(Units.Volts.of(GRAVITY_VOLTS));

    private double openLoopMotorSpeed = -0.2;
    private double currentSetpoint = 0;
    private boolean lazyHold;


    public Elevator() {
        right.setControl(new Follower(left.getDeviceID(), true));

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 2.5; //2.5
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kG = GRAVITY_VOLTS; // -0.75

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 40; //80;
        motionMagicConfigs.MotionMagicAcceleration = 60; // 140;
        motionMagicConfigs.MotionMagicJerk = 0;
        motionMagicConfigs.MotionMagicExpo_kA = 0.0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.01;

        TalonFXConfiguration configuration = new TalonFXConfiguration()
                .withSlot0(slot0)
                .withMotionMagic(motionMagicConfigs)
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(0.0)
                                .withReverseSoftLimitEnable(true)
                                .withReverseSoftLimitThreshold(-43)
                );

        left.getConfigurator().apply(configuration);

        // Zero motor position at startup
        left.setPosition(0);
        left.setControl(neutralOut);

        this.setDefaultCommand(new DefaultHoldPositionCommand(this));
    }

    @Override
    public void teleopInit() {
        this.currentSetpoint = this.getCurrentPosition();
    }

    private void setOpenLoop(double speed) {
        left.setControl(dutyCycleOut.withOutput(speed));
    }

    private void moveToPosition(ElevatorSetpoint setpoint) {
        moveToEncoderPosition(setpoint.val);
    }

    private void moveToEncoderPosition(double encoderPosition) {
        this.currentSetpoint = encoderPosition;
        left.setControl(motionMagicV.withPosition(encoderPosition));
    }

    /**
     * Returns the position of the elevator in motor rotations
     *
     * @return the position of the elevator in motor rotations
     */
    private double getCurrentPosition() {
        return left.getPosition().getValueAsDouble();
    }

    public boolean isInPosition() {
        return Math.abs(getCurrentPosition() - currentSetpoint) < ELEVATOR_POSITION_THRESHOLD;
    }

    void setLazyHold(boolean lazyHold) {
        this.lazyHold = lazyHold;
    }

    public Command openLoopCommand(Supplier<Double> speed) {
//        return this.run(() -> this.runOpenLoop(speed.get()));
        return this.run(() -> {
            System.out.println("Elevator runOpenLoop speed: " + speed.get());
            this.setOpenLoop(speed.get());
        }   );
    }

    public Command openLoopUpCommand() {
        return this.run(() -> this.setOpenLoop(openLoopMotorSpeed)).withName("Open Loop Up");
    }

    public Command openLoopDownCommand() {
        return this.run(() -> this.setOpenLoop(-openLoopMotorSpeed)).withName("Open Loop Down");
    }

    /**
     * Applies a neutral output to the elevator motors
     *
     * @return a command that applies a neutral output to the elevator motors
     */
    public Command openLoopStopCommand() {
        return this.runOnce(() -> left.setControl(neutralOut)).withName("Open Loop Stop");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("Current Position", this::getCurrentPosition, null);
        builder.addBooleanProperty("Is In Position", this::isInPosition, null);
        builder.addDoubleProperty("Current Setpoint", () -> currentSetpoint, this::moveToEncoderPosition);

        builder.addDoubleProperty("Open Loop Motor Speed", () -> openLoopMotorSpeed, (speed) -> openLoopMotorSpeed = speed);
        builder.addDoubleProperty("Left Motor Volts", () -> left.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Left Motor DutyCycle", () -> left.getDutyCycle().getValueAsDouble(), null);
        builder.addDoubleProperty("Left Motor Current", () -> left.getStatorCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Right Motor Current", () -> right.getStatorCurrent().getValueAsDouble(), null);
        builder.addBooleanProperty("Lazy Mode", () -> lazyHold, this::setLazyHold);
    }

    public enum ElevatorSetpoint { //TODO: these numbers will probably break things if ran on the bot but I need a robot built before we're able to fix them
        Zero(0),
        TROUGH(-16.5),
        L2(-17.25),
        L3(-26),
        L4(-39),
        AlgaeBarge(-38.5),
        AlgaeProcessor(0.0),
        AlgaeReefHigh(-14.0),
        AlgaeReefLow(-5.0);

        public final double val;

        ElevatorSetpoint(double val) {
            this.val = val;
        }
    }


    public static class ElevatorMoveToPositionCommand extends Command {
        private final ElevatorSetpoint setpoint;

        public ElevatorMoveToPositionCommand(ElevatorSetpoint setpoint) {
            this.setpoint = setpoint;
            addRequirements(Subsystems.elevator);
        }

        @Override
        public void initialize() {
            Subsystems.elevator.moveToPosition(this.setpoint);
        }

        @Override
        public boolean isFinished() {
            return Subsystems.elevator.isInPosition();
        }
    }

    public class DefaultHoldPositionCommand extends Command {
        public DefaultHoldPositionCommand(Elevator elevator) {
            addRequirements(elevator);
        }

        @Override
        public void initialize() {
            double currentPosition = Subsystems.elevator.getCurrentPosition();
            Subsystems.elevator.moveToEncoderPosition(currentPosition);
        }

        @Override
        public void execute() {
            if (Subsystems.elevator.isInPosition() &&
                    MathUtil.isNear(0.0, Subsystems.elevator.getCurrentPosition(), 0.25)) {
                Subsystems.elevator.left.setControl(neutralOut);
                Subsystems.elevator.setLazyHold(true);
            }
        }

        @Override
        public void end(boolean interrupted) {
            Subsystems.elevator.setLazyHold(false);
        }
    }
}