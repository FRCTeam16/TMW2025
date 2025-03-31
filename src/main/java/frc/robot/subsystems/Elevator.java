package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.amd.ElevatorAMDCommand;
import frc.robot.util.BSLogger;

import java.util.function.Supplier;

public class Elevator extends SubsystemBase implements Lifecycle, AMD<ElevatorAMDCommand.ElevatorDataCollector> {
    public static final double GRAVITY_VOLTS = -0.5;
    public static final double ELEVATOR_POSITION_THRESHOLD = 0.3;
    private final TalonFX left = new TalonFX(Robot.robotConfig.getCanID("elevatorLeftMotor"));
    private final TalonFX right = new TalonFX(Robot.robotConfig.getCanID("elevatorRightMotor"));

    private final NeutralOut neutralOut = new NeutralOut();
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicV = new MotionMagicVoltage(0)
            .withFeedForward(Units.Volts.of(GRAVITY_VOLTS));

    private double openLoopMotorSpeed = -0.2;
    private double currentSetpoint = 0;
    private ElevatorSetpoint requestedSetpoint = Elevator.ElevatorSetpoint.Zero;
    private boolean lazyHold;
    private double openLoopMax = 0.3;

    private double elevatorUpThreshold = -26.0;

    private Alert coralObstructionAlert = new Alert("Coral is obstructing elevator path", Alert.AlertType.kError);

    private final ElevatorTelemetry telemetry = new ElevatorTelemetry();

    public Elevator() {
        right.setControl(new Follower(left.getDeviceID(), true));

        Slot0Configs slot0 = new Slot0Configs()
                .withKP(2.5)
                .withKG(GRAVITY_VOLTS);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80) // 80
                .withMotionMagicAcceleration(160) // 140
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
        boolean isElevatorObstructed = isElevatorObstructedByCoral();
        if (isElevatorObstructed ^ coralObstructionAlert.get()) {
            coralObstructionAlert.set(isElevatorObstructed);
            telemetry.obstructionDetected.append(isElevatorObstructed);
        }
        telemetry.currentPosition.append(getCurrentPosition());
    }

    boolean isElevatorObstructedByCoral() {
        return Subsystems.coralIntake.coralDetectedByLaserCAN();
    }

    private void setOpenLoop(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, -openLoopMax, openLoopMax);
        left.setControl(dutyCycleOut.withOutput(clampedSpeed));
        telemetry.openLoopMotorSpeed.append(clampedSpeed);
    }

    private void moveToPosition(ElevatorSetpoint setpoint) {
        if (setpoint == null) {
            return;
        }
        BSLogger.log("Elevator", "moveToPosition: " + setpoint.name());
        requestedSetpoint = setpoint;
        moveToEncoderPosition(setpoint.val);
        telemetry.requestedSetpoint.append(setpoint.name());
    }

    private void moveToEncoderPosition(double encoderPosition) {
        BSLogger.log("Elevator", "moveToEncoderPosition: " + encoderPosition);
        this.currentSetpoint = encoderPosition;
        left.setControl(motionMagicV.withPosition(encoderPosition));
        telemetry.currentSetpoint.append(encoderPosition);
    }

    /**
     * Returns the position of the elevator in motor rotations
     */
    public double getCurrentPosition() {
        return left.getPosition().getValueAsDouble();
    }

    public ElevatorSetpoint getRequestedSetpoint() {
        return requestedSetpoint;
    }

    public boolean isInPosition() {
        final double currentPos = getCurrentPosition();
        final double error = Math.abs(currentPos - currentSetpoint);

        if (ElevatorSetpoint.Zero == requestedSetpoint) {
             if (lazyHold) {
                 return true;
             }
             if (error < 5) {
                 return true;
             }
        }

        boolean inPosition = error < ELEVATOR_POSITION_THRESHOLD && left.getVelocity().getValueAsDouble() < 0.1;
        telemetry.isInPosition.append(inPosition);
        return inPosition;
    }

    void setLazyHold(boolean lazyHold) {
        this.lazyHold = lazyHold;
    }

    public Command openLoopCommand(Supplier<Double> speed) {
        return this.run(() -> {
            double rawSpeed = speed.get();
            boolean isOpenLoop = left.getControlMode().getValue() == ControlModeValue.DutyCycleOut;
            if (MathUtil.isNear(0, Math.abs(rawSpeed), 0.05)) {
                // Only apply MM if we are in open loop to avoid slip
                if (isOpenLoop) {
                    Subsystems.elevator.moveToEncoderPosition(this.getCurrentPosition());
                }
            } else {
                this.setOpenLoop(speed.get());
            }
        }).withName("Elevator Manual Control");
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
        builder.addStringProperty("Requested", () -> requestedSetpoint != null ? requestedSetpoint.name() : "none",
                null);
        builder.addBooleanProperty("Coral Obstruction", this::isElevatorObstructedByCoral, null);


        if (Constants.DebugSendables.Elevator) {
            builder.addDoubleProperty("elevatorUpThreshold", () -> elevatorUpThreshold, (t) -> elevatorUpThreshold = t);
            builder.addBooleanProperty("Lazy Mode", () -> lazyHold, this::setLazyHold);

            builder.addDoubleProperty("Open Loop Motor Speed", () -> openLoopMotorSpeed,
                    (speed) -> openLoopMotorSpeed = speed);
            builder.addDoubleProperty("Open Loop Max", () -> openLoopMax, (max) -> openLoopMax = max);
            builder.addDoubleProperty("Left Motor Volts", () -> left.getMotorVoltage().getValueAsDouble(), null);
            builder.addDoubleProperty("Left Motor DutyCycle", () -> left.getDutyCycle().getValueAsDouble(), null);
            builder.addDoubleProperty("Left Motor Current", () -> left.getStatorCurrent().getValueAsDouble(), null);
            builder.addDoubleProperty("Right Motor Current", () -> right.getStatorCurrent().getValueAsDouble(), null);
        }
    }

    /**
     * Returns true if the elevator is above a default position
     */
    public boolean isElevatorUp() {
        return getCurrentPosition() < elevatorUpThreshold;
    }

    @Override
    public void collectAMDData(ElevatorAMDCommand.ElevatorDataCollector dataCollector) {
        dataCollector.collectData(isInPosition(), left.getStatorCurrent(), right.getStatorCurrent());
    }

    private boolean isRequestedZero() {
        return ElevatorSetpoint.Zero == Subsystems.elevator.requestedSetpoint;
    }

    public enum ElevatorSetpoint {
        Zero(0),
        TROUGH(-16.5),
        L2(-25.75),
        L3(-38),
        L4(-56.5),
        AlgaeBarge(-38.5),
        AlgaeProcessor(0.0),
        AlgaeReefHigh(-20.75),
        AlgaeReefLow(-7.75);

        public final double val;

        ElevatorSetpoint(double val) {
            this.val = val;
        }
    }

    public static class ElevatorMoveToPositionCommand extends Command {
        private final ElevatorSetpoint setpoint;
        private boolean abort = false;
        private boolean noWaitForFinish = false;

        public ElevatorMoveToPositionCommand(ElevatorSetpoint setpoint) {
            this.setpoint = setpoint;
            addRequirements(Subsystems.elevator);
        }

        @Override
        public void initialize() {
            this.abort = Subsystems.elevator.isElevatorObstructedByCoral();
            if (abort) {
                BSLogger.log("ElevatorMoveToPositionCommand", "Coral obstruction detected, setting abort");
                return;
            }
            Subsystems.elevator.moveToPosition(this.setpoint);
        }

        @Override
        public boolean isFinished() {
            if (abort || noWaitForFinish || Subsystems.elevator.isInPosition()) {
                BSLogger.log("ElevatorMoveToPositionCommand", "abort: " + abort + 
                " | noWaitForFinish: " + noWaitForFinish + " | inPosition: " + Subsystems.elevator.isInPosition());
                return true;
            }
            return false;
        }

        public ElevatorMoveToPositionCommand withNoWait() {
            this.noWaitForFinish = true;
            return this;
        }
    }

    public class DefaultHoldPositionCommand extends Command {
        public DefaultHoldPositionCommand(Elevator elevator) {
            addRequirements(elevator);
        }

        @Override
        public void initialize() {
            if (!isInPosition()) {
                double currentPosition = Subsystems.elevator.getCurrentPosition();
                Subsystems.elevator.moveToEncoderPosition(currentPosition);
                BSLogger.log("DefaultHoldPositionCommand", "Explicitly setting position to hold: " + currentPosition);
            }
        }

        @Override
        public void execute() {
            // If we are near the bottom and not moving, apply a neutral output
            if (!lazyHold && Subsystems.elevator.isInPosition() && isRequestedZero()) {
                Subsystems.elevator.setLazyHold(true);
                Subsystems.elevator.left.setControl(neutralOut);
            }
        }

        @Override
        public void end(boolean interrupted) {
            if (lazyHold && isRequestedZero()) {
//                Subsystems.elevator.left.setPosition(0);
            }
            Subsystems.elevator.setLazyHold(false);
        }
    }

    class ElevatorTelemetry {

        DoubleLogEntry leftMotorCurrent;
        DoubleLogEntry rightMotorCurrent;
        DoubleLogEntry currentPosition;
        DoubleLogEntry currentSetpoint;
        BooleanLogEntry isInPosition;
        StringLogEntry requestedSetpoint;
        BooleanLogEntry obstructionDetected;
        DoubleLogEntry openLoopMotorSpeed;

        ElevatorTelemetry() {
            DataLog log = DataLogManager.getLog();
            leftMotorCurrent = new DoubleLogEntry(log, "Telemetry/Elevator/LeftMotorCurrent");
            rightMotorCurrent = new DoubleLogEntry(log, "Telemetry/Elevator/RightMotorCurrent");
            currentPosition = new DoubleLogEntry(log, "Telemetry/Elevator/CurrentPosition");
            currentSetpoint = new DoubleLogEntry(log, "Telemetry/Elevator/CurrentSetpoint");
            isInPosition = new BooleanLogEntry(log, "Telemetry/Elevator/IsInPosition");
            requestedSetpoint = new StringLogEntry(log, "Telemetry/Elevator/RequestedSetpoint");
            obstructionDetected = new BooleanLogEntry(log, "Telemetry/Elevator/ObstructionDetected");
            openLoopMotorSpeed = new DoubleLogEntry(log, "Telemetry/Elevator/OpenLoopMotorSpeed");
        }
    }
}