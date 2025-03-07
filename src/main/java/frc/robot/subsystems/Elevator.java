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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;

import java.util.function.Supplier;

public class Elevator extends SubsystemBase implements Lifecycle {
    public static final double GRAVITY_VOLTS = -0.5;
    public static final double ELEVATOR_POSITION_THRESHOLD = 0.2;
    private final TalonFX left = new TalonFX(Robot.robotConfig.getCanID("elevatorLeftMotor"));
    private final TalonFX right = new TalonFX(Robot.robotConfig.getCanID("elevatorRightMotor"));

    private final NeutralOut neutralOut = new NeutralOut();
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicV = new MotionMagicVoltage(0).withFeedForward(Units.Volts.of(GRAVITY_VOLTS));

    private double openLoopMotorSpeed = -0.2;
    private double currentSetpoint = 0;
    private ElevatorSetpoint requestedSetpoint = Elevator.ElevatorSetpoint.Zero;
    private boolean lazyHold;
    private double openLoopMax = 0.3;

    private double elevatorUpThreshold = -6.0;

    private Alert coralObstructionAlert = new Alert("Coral is obstructing elevator path", Alert.AlertType.kError);



    public Elevator() {
        right.setControl(new Follower(left.getDeviceID(), true));

        Slot0Configs slot0 = new Slot0Configs()
                .withKP(2.5)
                .withKG(GRAVITY_VOLTS);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(77)  // 80
                .withMotionMagicAcceleration(110)     // 140
                .withMotionMagicJerk(0)
                .withMotionMagicExpo_kA(0.0)
                .withMotionMagicExpo_kV(0.01);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(0.0)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(-60);

        TalonFXConfiguration configuration = new TalonFXConfiguration()
                .withSlot0(slot0)
                .withMotionMagic(motionMagicConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

        left.getConfigurator().apply(configuration);

        // Zero motor position at startup
        left.setPosition(0);
        left.setControl(neutralOut);
        this.setDefaultCommand(new DefaultHoldPositionCommand(this));
    }

    @Override
    public void robotInit() {
        left.setPosition(0);
    }

    @Override
    public void teleopInit() {
        this.currentSetpoint = this.getCurrentPosition();
    }

    @Override
    public void periodic() {
        coralObstructionAlert.set(isElevatorObstructedByCoral());
    }

    boolean isElevatorObstructedByCoral() {
        return Subsystems.coralIntake.coralDetectedAtTopSensor() &&
                !Subsystems.coralIntake.coralDetectedAtBottomSensor();
    }

    private void setOpenLoop(double speed) {
        double clampedSpeed =MathUtil.clamp(speed, -openLoopMax, openLoopMax);
        left.setControl(dutyCycleOut.withOutput(clampedSpeed));
    }

    private void moveToPosition(ElevatorSetpoint setpoint) {
        requestedSetpoint = setpoint;
        moveToEncoderPosition(setpoint.val);
    }

    private void moveToEncoderPosition(double encoderPosition) {
        BSLogger.log("Elevator", "Moving to position: " + encoderPosition);
//        if (!MathUtil.isNear(0, encoderPosition, 0.05)) {
//            encoderPosition += encoderOffset;
//            BSLogger.log("Elevator", "Moving to adjusted position: " + encoderPosition);
//        }
        this.currentSetpoint = encoderPosition;
        left.setControl(motionMagicV.withPosition(encoderPosition));
    }

    /**
     * Returns the position of the elevator in motor rotations
     */
    public double getCurrentPosition() {
        return left.getPosition().getValueAsDouble();
    }

    public boolean isInPosition() {
        double currentPos = getCurrentPosition();
        double error = Math.abs(currentPos - currentSetpoint);
        return error < ELEVATOR_POSITION_THRESHOLD &&
                left.getVelocity().getValueAsDouble() < 0.1;
    }

    void setLazyHold(boolean lazyHold) {
        this.lazyHold = lazyHold;
    }

    public Command openLoopCommand(Supplier<Double> speed) {
        return this.run(() -> this.setOpenLoop(speed.get())).withName("Elevator Manual Control");
    }

    @Deprecated
    public Command openLoopUpCommand() {
        return this.run(() -> this.setOpenLoop(openLoopMotorSpeed)).withName("Open Loop Up");
    }

    @Deprecated
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
        builder.addStringProperty("Requested", () -> requestedSetpoint != null ? requestedSetpoint.name() : "none", null);
        builder.addBooleanProperty("Coral Obstruction", this::isElevatorObstructedByCoral, null);
//        builder.addDoubleProperty("Encoder Offset", () -> encoderOffset, (v) -> encoderOffset = v);

        builder.addDoubleProperty("Open Loop Motor Speed", () -> openLoopMotorSpeed, (speed) -> openLoopMotorSpeed = speed);
        builder.addDoubleProperty("Open Loop Max", () -> openLoopMax, (max) -> openLoopMax = max);
        builder.addDoubleProperty("Left Motor Volts", () -> left.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Left Motor DutyCycle", () -> left.getDutyCycle().getValueAsDouble(), null);
        builder.addDoubleProperty("Left Motor Current", () -> left.getStatorCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Right Motor Current", () -> right.getStatorCurrent().getValueAsDouble(), null);
        builder.addBooleanProperty("Lazy Mode", () -> lazyHold, this::setLazyHold);
        builder.addDoubleProperty("elevatorUpThreshold", () -> elevatorUpThreshold, (t) -> elevatorUpThreshold = t);
    }

    /**
     * Returns true if the elevator is above a default position
     */
    public boolean isElevatorUp() {
        return getCurrentPosition() < elevatorUpThreshold;
    }

    public enum ElevatorSetpoint {
        Zero(0),
        TROUGH(-16.5),
        L2(-25.75),
        L3(-38),
        L4(-56.5),
        AlgaeBarge(-38.5),
        AlgaeProcessor(0.0),
        AlgaeReefHigh(-20),
        AlgaeReefLow(-7.0);

        public final double val;

        ElevatorSetpoint(double val) {
            this.val = val;
        }
    }


    public static class ElevatorMoveToPositionCommand extends Command {
        private final ElevatorSetpoint setpoint;
        private boolean abort = false;

        public ElevatorMoveToPositionCommand(ElevatorSetpoint setpoint) {
            this.setpoint = setpoint;
            addRequirements(Subsystems.elevator);
        }

        @Override
        public void initialize() {
            if (Subsystems.elevator.isElevatorObstructedByCoral()) {
                BSLogger.log("ElevatorMoveToPositionCommand", "Coral obstruction detected, setting abort");
                abort = true;
                return;
            }
            Subsystems.elevator.moveToPosition(this.setpoint);
        }

        @Override
        public boolean isFinished() {
            return abort || Subsystems.elevator.isInPosition();
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
            // If we are near the bottom and not moving, apply a neutral output to the motors
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